#include <Arduino.h>
#include "config.h"
#include <RosNodeManager.h>
#include "RosWiFiHelper.h"

// ==== 腳位定義 ====
#define IN1 26
#define IN2 27
#define SERVO_PIN 25
#define TRI_MOTOR_PIN 19
#define BAT1_PIN 32
#define BAT2_PIN 33

// ==== PWM 通道定義 ====
#define SERVO_CH 2
#define MOTOR_CH1 0
#define MOTOR_CH2 1
#define TRI_MOTOR_CH 3

// ==== PWM 參數設定 ====
const int PWM_FREQ_DC    = 5000;
const int PWM_FREQ_SERVO = 50;
const int PWM_RES_DC     = 8;
const int PWM_RES_SERVO  = 16;     // 16-bit (0..65535)
const int PWM_PERIOD     = 20000;  // us, for 50Hz

// ==== 狀態 ====
static int lastDCDir = 0;   // -1, 0, +1
static int lastTriDir = 0;  // 保留用（目前 ESC 無正反）

// ==== 目標命令（由 ROS 訂閱寫入，loop 套用）====
static volatile int  cmd_dc_percent   = 0;   // -100..100
static volatile int  cmd_servo_deg    = 90;  // 0..180
static volatile int  cmd_esc_percent  = 0;   // 0..100
static volatile bool cmd_stop_all     = false;

static volatile bool dirty_dc   = false;
static volatile bool dirty_srv  = false;
static volatile bool dirty_esc  = false;
static volatile bool dirty_stop = false;

// ==== 工具函式 ====
void stopMotorSoft() {
  ledcDetachPin(IN1);
  ledcDetachPin(IN2);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(20);
  ledcAttachPin(IN1, MOTOR_CH1);
  ledcAttachPin(IN2, MOTOR_CH2);
}

void stopTriMotorSoft() {
  ledcWrite(TRI_MOTOR_CH, 0);
  delay(20);
}

void stopAll() {
  stopMotorSoft();
  stopTriMotorSoft();
  ledcWrite(SERVO_CH, 0);
  lastDCDir = 0;
  lastTriDir = 0;
}

// 實際套用命令：DC 馬達（-100..100）
void apply_dc_percent(int percent) {
  percent = constrain(percent, -100, 100);

  if (percent == 0) {
    stopMotorSoft();
    lastDCDir = 0;
    return;
  }

  if (percent > 0) {
    if (lastDCDir != 1) {
      stopMotorSoft(); // 避免硬反轉
      // 不在回呼中 delay，這裡在 loop 裡短暫 delay 可接受
      delay(50);
    }
    int pwm = map(percent, 0, 100, 0, 255);
    ledcWrite(MOTOR_CH1, pwm);
    ledcWrite(MOTOR_CH2, 0);
    lastDCDir = 1;
  } else { // percent < 0
    if (lastDCDir != -1) {
      stopMotorSoft();
      delay(50);
    }
    int pwm = map(-percent, 0, 100, 0, 255);
    ledcWrite(MOTOR_CH1, 0);
    ledcWrite(MOTOR_CH2, pwm);
    lastDCDir = -1;
  }
}

// 實際套用命令：舵機角度（0..180）
void apply_servo_deg(int deg) {
  int angle = constrain(deg, 0, 180);
  int us = map(angle, 0, 180, 500, 2500);
  int duty = (us * 65536L) / PWM_PERIOD;  // 16bit
  ledcWrite(SERVO_CH, duty);
}

// 實際套用命令：ESC 百分比（0..100 → 1000..2000us）
void apply_esc_percent(int percent) {
  int p = constrain(percent, 0, 100);
  int us = map(p, 0, 100, 1000, 2000);
  int duty = (us * 65536L) / PWM_PERIOD;
  ledcWrite(TRI_MOTOR_CH, duty);
}

// ==== ROS 回呼（只更新命令，不做阻塞動作）====
void cb_dc_percent(const std_msgs__msg__Int32* m, void*) {
  cmd_dc_percent = m->data;
  dirty_dc = true;
}

void cb_servo_deg(const std_msgs__msg__Int32* m, void*) {
  cmd_servo_deg = m->data;
  dirty_srv = true;
}

void cb_esc_percent(const std_msgs__msg__Int32* m, void*) {
  cmd_esc_percent = m->data;
  dirty_esc = true;
}

void cb_stop(const std_msgs__msg__Bool* m, void*) {
  if (m->data) {
    cmd_stop_all = true;
    dirty_stop = true;
  }
}

// ==== Node Manager ====
static RosNodeManager* g_node = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);

  // 硬體初始化
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(BAT1_PIN, INPUT);
  pinMode(BAT2_PIN, INPUT);

  analogReadResolution(12);

  ledcSetup(MOTOR_CH1, PWM_FREQ_DC,    PWM_RES_DC);
  ledcSetup(MOTOR_CH2, PWM_FREQ_DC,    PWM_RES_DC);
  ledcAttachPin(IN1, MOTOR_CH1);
  ledcAttachPin(IN2, MOTOR_CH2);

  ledcSetup(SERVO_CH, PWM_FREQ_SERVO,  PWM_RES_SERVO);
  ledcAttachPin(SERVO_PIN, SERVO_CH);

  ledcSetup(TRI_MOTOR_CH, PWM_FREQ_SERVO, PWM_RES_SERVO);
  ledcAttachPin(TRI_MOTOR_PIN, TRI_MOTOR_CH);

  stopAll();

  // 連線（依你的 helper）
  setup_wifi_fallback();

  // micro-ROS
  g_node = new RosNodeManager();
  if (!g_node->init(NODE_NAME)) {
    Serial.println("RosNodeManager init failed!");
    while (1) delay(1000);
  }

  // === 訂閱：用 ROS 直接控制 ===
  g_node->add_int32_subscription ("/ctrl/dc_percent",   &cb_dc_percent);
  g_node->add_int32_subscription ("/ctrl/servo_deg",    &cb_servo_deg);
  g_node->add_int32_subscription ("/ctrl/esc_percent",  &cb_esc_percent);
  g_node->add_bool_subscription   ("/ctrl/stop",        &cb_stop);

  Serial.println("ROS control ready.");
}

void loop() {
  if (g_node) g_node->spin_some(20);

  // 在 loop 中套用命令（避免在回呼中 delay/阻塞）
  if (dirty_stop && cmd_stop_all) {
    stopAll();
    cmd_stop_all = false;
    dirty_stop = false;
    Serial.println("[ROS] STOP ALL");
  }

  if (dirty_dc) {
    apply_dc_percent(cmd_dc_percent);
    dirty_dc = false;
    // Serial.printf("[ROS] DC %d%%\n", cmd_dc_percent);
  }

  if (dirty_srv) {
    apply_servo_deg(cmd_servo_deg);
    dirty_srv = false;
    // Serial.printf("[ROS] SERVO %d deg\n", cmd_servo_deg);
  }

  if (dirty_esc) {
    apply_esc_percent(cmd_esc_percent);
    dirty_esc = false;
    // Serial.printf("[ROS] ESC %d%%\n", cmd_esc_percent);
  }

  delay(10);
}
