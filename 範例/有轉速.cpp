#include <Arduino.h>
#include <Wire.h>
#include <AM232X.h>

#include "config.h"
#include <RosNodeManager.h>
#include "RosWiFiHelper.h"

// ──────────────────────────────────────────────────────────────
// I2C: AM232X
AM232X am;
static float g_temp_c   = NAN;
static float g_humi_pct = NAN;

static uint32_t last_read_ms = 0;
static const uint32_t READ_PERIOD_MS = 2000;

static void sensor_update() {
  const uint32_t now = millis();
  if (now - last_read_ms >= READ_PERIOD_MS) {
    if (am.read() == AM232X_OK) {
      g_temp_c   = am.getTemperature();
      g_humi_pct = am.getHumidity();
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
      Serial.printf("[AM232X] T=%.1fC, RH=%.1f%%\n", g_temp_c, g_humi_pct);
#endif
    } else {
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
      Serial.println("[AM232X] read FAIL");
#endif
    }
    last_read_ms = now;
  }
}
static float read_temp_c()   { return g_temp_c; }
static float read_humi_pct() { return g_humi_pct; }

// ──────────────────────────────────────────────────────────────
// LED（位元控制）
#define LED_A_PIN  25
#define LED_B_PIN  33
#define LED_C_PIN  32

void on_led_cmd(const std_msgs__msg__Int32* msg, void*) {
  int value = msg->data;
  bool ledA = value & 0b001;
  bool ledB = value & 0b010;
  bool ledC = value & 0b100;
  digitalWrite(LED_A_PIN, ledA);
  digitalWrite(LED_B_PIN, ledB);
  digitalWrite(LED_C_PIN, ledC);
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[LED] cmd=%d -> A=%d, B=%d, C=%d\n", value, ledA, ledB, ledC);
#endif
}

// ──────────────────────────────────────────────────────────────
// 馬達：PWM (Servo) → BLHeli_32（3D 模式） + 平滑 ramp
static const int MOTOR_PWM_PIN = 18;
static const int PWM_CHANNEL   = 0;
static const int PWM_FREQ_HZ   = 50;        // 50Hz -> 20ms
static const int PWM_RES_BITS  = 16;

// 平滑控制參數
static const int   RAMP_PERIOD_MS = 20;     // 每 20ms 更新一次
static float       RAMP_US_PER_S  = 800.0f; // 每秒最多變化 800 μs（可調）
static uint32_t    g_last_ramp_ms = 0;

static volatile int g_motor_us = 1500;        // 實際輸出 μs（被平滑後）
static volatile int g_target_us = 1500;       // 目標 μs（回調設定）
static volatile int g_motor_percent = 0;      // -100 ~ +100（顯示用）

static float read_motor_us()      { return (float)g_motor_us; }
static float read_motor_percent() { return (float)g_motor_percent; }

// μs → duty
static inline uint32_t us_to_duty(uint32_t us) {
  const uint32_t max_count = (1u << PWM_RES_BITS) - 1u;
  return (uint32_t)(((double)us / 20000.0) * (double)max_count); // 20ms 週期
}

static void apply_motor_output_us_now(int us) {
  us = constrain(us, 1000, 2000);
  ledcWrite(PWM_CHANNEL, us_to_duty((uint32_t)us));
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[MOTOR] apply us=%d (1500=stop)\n", us);
#endif
}

static void motor_ramp_update() {
  const uint32_t now = millis();
  if (now - g_last_ramp_ms < RAMP_PERIOD_MS) return;

  const float dt_s = (now - g_last_ramp_ms) / 1000.0f;
  g_last_ramp_ms = now;

  int current = g_motor_us;
  int target  = constrain(g_target_us, 1000, 2000);
  if (current == target) return;

  int max_step = (int)ceilf(RAMP_US_PER_S * dt_s);
  if (max_step < 1) max_step = 1;

  int delta = target - current;
  if (delta > 0) current += min(delta, max_step);
  else           current -= min(-delta, max_step);

  g_motor_us = constrain(current, 1000, 2000);
  apply_motor_output_us_now(g_motor_us);
}

static inline void set_target_us(int us) {
  g_target_us = constrain(us, 1000, 2000);
  g_motor_percent = map(g_target_us, 1000, 2000, -100, 100);
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[MOTOR] target_us=%d, percent=%d\n", g_target_us, g_motor_percent);
#endif
}

// 訂閱：直接 μs（1000~2000）
void on_motor_us(const std_msgs__msg__Int32* msg, void*) {
  set_target_us(msg->data);
}

// 訂閱：-100..+100 → 1500±500 μs
void on_motor_percent(const std_msgs__msg__Int32* msg, void*) {
  int pct = constrain(msg->data, -100, 100);
  g_motor_percent = pct;
  int us = 1500 + (pct * 5);  // ±100 → ±500 μs
  set_target_us(us);
}

// 動態調整平滑速度（us/s）
void on_motor_ramp_rate(const std_msgs__msg__Int32* msg, void*) {
  int v = max(50, msg->data);
  RAMP_US_PER_S = (float)v;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[MOTOR] ramp rate = %.1f us/s\n", RAMP_US_PER_S);
#endif
}

// ──────────────────────────────────────────────────────────────
// 霍爾轉速（1 感測器 + 2 磁鐵）
static const int  HALL_PIN         = 27;      // GPIO for digital Hall output
static const int  PULSES_PER_REV   = 2;       // 兩顆磁鐵 → 2 脈衝/轉
static const uint32_t MIN_INTERVAL_US = 1000; // 防抖/抗雜訊最短間隔 1ms

static volatile uint32_t g_pulse_count = 0;   // 累積脈衝
static volatile uint32_t g_last_edge_us = 0;  // 上一個有效邊緣時間
static float g_rpm = 0.0f;                    // 平滑後 RPM
static float g_pps = 0.0f;                    // pulses per second（除以 PPR 可得 RPS）
static const float RPM_ALPHA = 0.3f;          // 指數平滑係數（0~1）

// 計算視窗
static const uint32_t RPM_WINDOW_MS = 250;    // 每 250ms 更新一次
static uint32_t g_last_rpm_calc_ms = 0;
static uint32_t g_last_pulse_snapshot = 0;

// ISR：上升緣計數 + 去抖（最短間隔）
void IRAM_ATTR hall_isr() {
  uint32_t now = micros();
  uint32_t dt = now - g_last_edge_us;
  if (dt >= MIN_INTERVAL_US) {
    g_last_edge_us = now;
    g_pulse_count++;
  }
}

static float read_rpm() { return g_rpm; }
static float read_pps() { return g_pps; }

static void hall_update() {
  const uint32_t now_ms = millis();
  if (now_ms - g_last_rpm_calc_ms < RPM_WINDOW_MS) return;

  uint32_t window_ms = now_ms - g_last_rpm_calc_ms;
  g_last_rpm_calc_ms = now_ms;

  // 取快照避免長時間關中斷
  uint32_t pulses = g_pulse_count;
  uint32_t delta  = pulses - g_last_pulse_snapshot;
  g_last_pulse_snapshot = pulses;

  // 視窗內脈衝率 → pps
  float window_s = window_ms / 1000.0f;
  float pps_now = (window_s > 0) ? (delta / window_s) : 0.0f;
  float rps_now = (PULSES_PER_REV > 0) ? (pps_now / (float)PULSES_PER_REV) : 0.0f;
  float rpm_now = rps_now * 60.0f;

  // 指數平滑
  g_pps = (1.0f - RPM_ALPHA) * g_pps + RPM_ALPHA * pps_now;
  g_rpm = (1.0f - RPM_ALPHA) * g_rpm + RPM_ALPHA * rpm_now;

#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[HALL] pulses=%u, pps=%.1f, rpm=%.1f\n", pulses, g_pps, g_rpm);
#endif
}

// ──────────────────────────────────────────────────────────────
// micro-ROS Node
RosNodeManager node;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);
  setup_wifi_fallback();

  if (!node.init(NODE_NAME)) {
    Serial.println("ROS 初始化失敗");
    return;
  }

  // LED
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(LED_C_PIN, OUTPUT);
  digitalWrite(LED_A_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  digitalWrite(LED_C_PIN, LOW);

  // 馬達 PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);

  g_motor_us   = 1500;
  g_target_us  = 1500;
  g_last_ramp_ms = millis();
  apply_motor_output_us_now(g_motor_us);

  // ── 霍爾輸入 ───────────────────────────
  pinMode(HALL_PIN, INPUT_PULLUP);                 // 數位霍爾→開啟內部上拉
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_isr, RISING);

  // ROS 訂閱/發佈設定
  node.add_int32_subscription("/cmd/led", &on_led_cmd);
  node.add_int32_subscription("/cmd/motor_us", &on_motor_us);
  node.add_int32_subscription("/cmd/motor_percent", &on_motor_percent);
  // node.add_int32_subscription("/cmd/motor_ramp", &on_motor_ramp_rate);

  node.add_float32_publisher("/env/temperature_c", 1000, &read_temp_c);
  node.add_float32_publisher("/env/humidity_pct", 1000, &read_humi_pct);

  // 轉速與脈衝率（200ms）
  node.add_float32_publisher("/motor/rpm", 200, &read_rpm);
  node.add_float32_publisher("/motor/pps", 200, &read_pps);

  Serial.println("✅ Ready: PWM ESC + Env Sensors + Hall RPM(2 magnets)");
}

void loop() {
  sensor_update();
  motor_ramp_update();  // 平滑更新輸出
  hall_update();        // 更新轉速估計
  node.spin_some(50);
}
