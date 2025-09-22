#include <Arduino.h>
#include <Wire.h>
#include <AM232X.h>

#include "config.h"
#include <RosPublisher.h>
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
      g_temp_c  = am.getTemperature();
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

// ─── ROS 發布器 ───────────────────────────────────────────────────────────
RosPublisherManager pubman;

static float read_temp_c() { return g_temp_c; }
static float read_humi_pct() { return g_humi_pct; }
// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);

  setup_wifi_fallback();
  if (!pubman.init(NODE_NAME)) {
    Serial.println("ROS 初始化失敗");
    return;
  }

  // 主題名稱可依需調整
  pubman.add_float32_publisher("/env/temperature_c", 1000, &read_temp_c);
  pubman.add_float32_publisher("/env/humidity_pct",   1000, &read_humi_pct);

  Serial.println("準備完畢。");
}

void loop() {
  sensor_update();       // 先更新感測值
  pubman.spin_some(50);  // 發布/處理 ROS 訊息
}
