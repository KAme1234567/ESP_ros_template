#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include <RosNodeManager.h>
#include "RosWiFiHelper.h"

// ──────────────────────────────────────────────────────────────
// 腳位
#define IN1           26     // 抽水馬達 H-bridge IN1
#define IN2           27     // 抽水馬達 H-bridge IN2
#define SERVO_PIN     25     // 舵機
#define MOTOR_PWM_PIN 19     // 主推 ESC (PWM 1000~2000us)
#define HALL_PIN      34     // 霍爾感測器
#define BUZZ_PIN      23     // 蜂鳴器（被動/有源皆可；此處以被動為主）

// ──────────────────────────────────────────────────────────────
// PWM 通道（避免互相衝突）
#define ESC_CH     0
#define SERVO_CH   1
#define PUMP_CH1   2
#define PUMP_CH2   3
#define BUZZ_CH    4   // 蜂鳴器專用通道（LEDC）

// 頻率/解析度
const int PWM_FREQ_DC     = 5000; // 抽水馬達 DC PWM
const int PWM_RES_DC      = 8;    // 0..255
const int PWM_FREQ_SERVO  = 50;   // 舵機/ESC 50Hz
const int PWM_RES_SERVO   = 16;   // 0..65535
const int PWM_PERIOD_US   = 20000;

// 蜂鳴器參數（被動蜂鳴器以方波驅動）
const int BUZZ_RES_BITS   = 10;   // 0..1023
const int BUZZ_DEF_FREQ   = 2000; // 2 kHz 默認音調
const int BUZZ_DUTY_ON    = 512;  // 50% 佔空比

// ──────────────────────────────────────────────────────────────
// ESC 緩升控制（50Hz，1500=停，1000~2000us）
static const int   RAMP_PERIOD_MS = 20;
static float       RAMP_US_PER_S  = 800.0f;
static uint32_t    g_last_ramp_ms = 0;
static volatile int g_motor_us    = 1500;
static volatile int g_target_us   = 1500;
static volatile int g_motor_percent = 0;

static inline uint32_t us_to_duty_servo(uint32_t us) {
  const uint32_t maxc = (1u << PWM_RES_SERVO) - 1u;
  return (uint32_t)((uint64_t)us * maxc / PWM_PERIOD_US);
}
static inline void esc_apply_now_us(int us) {
  us = constrain(us, 1000, 2000);
  ledcWrite(ESC_CH, us_to_duty_servo((uint32_t)us));
}
static inline void esc_set_target_us(int us) {
  g_target_us = constrain(us, 1000, 2000);
  g_motor_percent = map(g_target_us, 1000, 2000, -100, 100);
}
static void esc_ramp_update() {
  const uint32_t now = millis();
  if (now - g_last_ramp_ms < RAMP_PERIOD_MS) return;
  const float dt = (now - g_last_ramp_ms) / 1000.0f;
  g_last_ramp_ms = now;

  int cur = g_motor_us;
  int tar = g_target_us;
  if (cur == tar) return;

  int max_step = (int)ceilf(RAMP_US_PER_S * dt);
  if (max_step < 1) max_step = 1;

  int d = tar - cur;
  if (d > 0) cur += min(d, max_step);
  else       cur -= min(-d, max_step);

  g_motor_us = constrain(cur, 1000, 2000);
  esc_apply_now_us(g_motor_us);
}

// ROS: ESC 控制
void on_motor_us(const std_msgs__msg__Int32* m, void*)       { esc_set_target_us(m->data); }
void on_motor_percent(const std_msgs__msg__Int32* m, void*)  { 
  int pct = constrain(m->data, -100, 100); 
  g_motor_percent = pct; 
  esc_set_target_us(1500 + pct * 5);
}

// ──────────────────────────────────────────────────────────────
// 抽水馬達（H-bridge 2 線，5kHz PWM）：-100..100
static volatile int  cmd_pump_percent = 0;
static volatile bool dirty_pump = false;
static int lastPumpDir = 0; // -1/0/+1

// 軟停止：暫時脫鉤 PWM → 拉低 → 再掛回通道，避免 shoot-through
static void stopPumpSoft() {
  ledcDetachPin(IN1);
  ledcDetachPin(IN2);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(20);
  ledcAttachPin(IN1, PUMP_CH1);
  ledcAttachPin(IN2, PUMP_CH2);
  lastPumpDir = 0;
}

// 實際套用百分比
static void apply_pump_percent(int percent) {
  percent = constrain(percent, -100, 100);
  if (percent == 0) { stopPumpSoft(); return; }

  if (percent > 0) {
    if (lastPumpDir != 1) { stopPumpSoft(); delay(50); }
    int pwm = map(percent, 0, 100, 0, 255);
    ledcWrite(PUMP_CH1, pwm);
    ledcWrite(PUMP_CH2, 0);
    lastPumpDir = 1;
  } else { // < 0
    if (lastPumpDir != -1) { stopPumpSoft(); delay(50); }
    int pwm = map(-percent, 0, 100, 0, 255);
    ledcWrite(PUMP_CH1, 0);
    ledcWrite(PUMP_CH2, pwm);
    lastPumpDir = -1;
  }
}

// ROS 回呼（兩個主題都支援：你的新舊程式都能控）
void cb_pump_percent_cmd(const std_msgs__msg__Int32* m, void*) { cmd_pump_percent = m->data; dirty_pump = true; }
void cb_dc_percent_alias(const std_msgs__msg__Int32* m, void*) { cmd_pump_percent = m->data; dirty_pump = true; }

// ──────────────────────────────────────────────────────────────
// 舵機（50Hz）：角度/μs 兩種
static const int SERVO_MIN_US = 1000;
static const int SERVO_MAX_US = 2000;
static volatile int  cmd_servo_deg = 90;
static volatile bool dirty_srv = false;

static inline void servo_write_us(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(SERVO_CH, us_to_duty_servo((uint32_t)us));
}
void on_servo_deg(const std_msgs__msg__Int32* m, void*) { cmd_servo_deg = m->data; dirty_srv = true; }
void on_servo_us (const std_msgs__msg__Int32* m, void*) { servo_write_us(m->data); }

// ──────────────────────────────────────────────────────────────
// 霍爾轉速
static float g_rpm = 0.0f, g_pps = 0.0f;
static volatile uint32_t g_pulse_count = 0, g_last_edge_us = 0;
static const int  PULSES_PER_REV = 2;
static const uint32_t MIN_INTERVAL_US = 1000;
static const float RPM_ALPHA = 0.3f;
static const uint32_t RPM_WINDOW_MS = 250;
static uint32_t g_last_rpm_calc_ms = 0, g_last_pulse_snapshot = 0;

void IRAM_ATTR hall_isr() {
  uint32_t now = micros();
  uint32_t dt  = now - g_last_edge_us;
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
  uint32_t win = now_ms - g_last_rpm_calc_ms; g_last_rpm_calc_ms = now_ms;

  uint32_t pulses = g_pulse_count;
  uint32_t delta  = pulses - g_last_pulse_snapshot;
  g_last_pulse_snapshot = pulses;

  float win_s = win / 1000.0f;
  float pps_now = (win_s > 0) ? (delta / win_s) : 0.0f;
  float rps_now = (PULSES_PER_REV > 0) ? (pps_now / (float)PULSES_PER_REV) : 0.0f;
  float rpm_now = rps_now * 60.0f;

  g_pps = (1 - RPM_ALPHA) * g_pps + RPM_ALPHA * pps_now;
  g_rpm = (1 - RPM_ALPHA) * g_rpm + RPM_ALPHA * rpm_now;
}

// ──────────────────────────────────────────────────────────────
// 蜂鳴器（被動）：LEDC 產生音調
static inline void buzz_on(int freq_hz = BUZZ_DEF_FREQ) {
  // 設定頻率並給一個非零 duty
  ledcWriteTone(BUZZ_CH, freq_hz);
  ledcWrite(BUZZ_CH, BUZZ_DUTY_ON);
}
static inline void buzz_off() {
  ledcWrite(BUZZ_CH, 0);
}
static void beep_blocking(int count, int on_ms = 120, int off_ms = 120, int freq_hz = BUZZ_DEF_FREQ) {
  count = max(1, count);
  for (int i = 0; i < count; ++i) {
    buzz_on(freq_hz);
    delay(on_ms);
    buzz_off();
    if (i < count - 1) delay(off_ms);
  }
}

// ──────────────────────────────────────────────────────────────
RosNodeManager node;

void setup() {
  Serial.begin(115200);
  delay(100);

  // 硬體初始化
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // PWM 配置
  ledcSetup(ESC_CH,    PWM_FREQ_SERVO, PWM_RES_SERVO);
  ledcAttachPin(MOTOR_PWM_PIN, ESC_CH);
  esc_apply_now_us(1500);
  g_last_ramp_ms = millis();

  ledcSetup(SERVO_CH,  PWM_FREQ_SERVO, PWM_RES_SERVO);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
  servo_write_us(1500);

  ledcSetup(PUMP_CH1,  PWM_FREQ_DC,    PWM_RES_DC);
  ledcSetup(PUMP_CH2,  PWM_FREQ_DC,    PWM_RES_DC);
  ledcAttachPin(IN1, PUMP_CH1);
  ledcAttachPin(IN2, PUMP_CH2);
  stopPumpSoft();

  // 蜂鳴器 LEDC
  ledcSetup(BUZZ_CH, BUZZ_DEF_FREQ, BUZZ_RES_BITS);
  ledcAttachPin(BUZZ_PIN, BUZZ_CH);
  buzz_off();

  // 霍爾
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_isr, RISING);

  // ── 聯網提示：開始嘗試聯網 → 嗶 1 聲
  
  while (WiFi.status() == WL_CONNECTED) WiFi.disconnect(true);{
  // 網路 / micro-ROS
  beep_blocking(1, 120, 120, 2000);
  setup_wifi_fallback();
    if (!node.init(NODE_NAME)) {
      // 失敗提示：嗶 3 聲（較長）
      beep_blocking(3, 220, 140, 1400);
      Serial.println("ROS init failed");
    }
    delay(200);
  }


  // ROS 訂閱
  node.add_int32_subscription("/power/cmd/motor_us",      &on_motor_us);
  node.add_int32_subscription("/power/cmd/motor_percent", &on_motor_percent);

  node.add_int32_subscription("/power/cmd/pump_percent",  &cb_pump_percent_cmd); // 新
  node.add_int32_subscription("/power/ctrl/dc_percent",   &cb_dc_percent_alias); // 相容舊程式

  node.add_int32_subscription("/power/cmd/servo_deg",     &on_servo_deg);
  node.add_int32_subscription("/power/cmd/servo_us",      &on_servo_us);

  // ROS 發佈
  node.add_float32_publisher("/power/motor/rpm", 200, &read_rpm);
  node.add_float32_publisher("/power/motor/pps", 200, &read_pps);

  Serial.println("✅ Ready: ESC + Servo + Pump(5kHz H-bridge) + Hall RPM + Buzzer");

  // ── 全部就緒提示：嗶 2 聲
  beep_blocking(2, 120, 120, 2200);
}

void loop() {
  // 非阻塞：在 loop 裡做需要 delay 的事
  if (dirty_pump) {
    apply_pump_percent(cmd_pump_percent);
    dirty_pump = false;
  }
  if (dirty_srv) {
    int deg = constrain(cmd_servo_deg, 0, 180);
    int us  = map(deg, 0, 180, 1000, 2000);
    servo_write_us(us);
    dirty_srv = false;
  }

  esc_ramp_update();
  hall_update();
  node.spin_some(20);
}
