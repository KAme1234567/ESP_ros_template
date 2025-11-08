#include <Arduino.h>
#include <Wire.h>
#include <AM232X.h>

#include "config.h"
#include <RosNodeManager.h>   // ⭐ 改用這個
#include "RosWiFiHelper.h"

// ─── AM232X ──────────────────────────────────────────────────────────────
AM232X am;                 // I2C 位址固定 0x5C
static float g_temp_c = NAN;
static float g_humi_pct = NAN;

static uint32_t last_read_ms = 0;
static const uint32_t READ_PERIOD_MS = 2000;  // 感測器讀取頻率

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
      Serial.println("[AM232X] 讀取失敗");
#endif
    }
    last_read_ms = now;
  }
}

// ─── 讀取函式（給 Publisher 綁定） ───────────────────────────────
static float read_temp_c()  { return g_temp_c; }
static float read_humi_pct() { return g_humi_pct; }

// ─── LED 控制（位元解析） ─────────────────────────────────────────
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

  Serial.printf("[LED] cmd=%d -> A=%d, B=%d, C=%d\n", value, ledA, ledB, ledC);
}

// ─── Node Manager ────────────────────────────────────────────────────────
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

  // LED 腳位
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(LED_C_PIN, OUTPUT);
  digitalWrite(LED_A_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  digitalWrite(LED_C_PIN, LOW);

  // === 發布端 ===
  node.add_float32_publisher("/env/temperature_c", 1000, &read_temp_c);
  node.add_float32_publisher("/env/humidity_pct",   1000, &read_humi_pct);

  // === 訂閱端（十進制→二進制控制 3 顆 LED） ===
  node.add_int32_subscription("/cmd/led", &on_led_cmd);

  Serial.println("準備完畢。");
}

void loop() {
  sensor_update();
  node.spin_some(50);  // ⭐ 用 node（取代原本 pubman）
}
