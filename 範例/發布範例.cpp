#include <Arduino.h>
#include "config.h"
#include <RosPublisher.h>
#include "RosWiFiHelper.h"

// ===== providers =====
int32_t read_battery_mv() {
  return 11700; // 11.7V (mV)
}

float read_depth_m() {
  static float d = 0.0f;
  d += 0.01f; // 模擬深度累加
  return d;   // 公尺
}

bool read_is_autonomous() {
  // 例：依系統狀態回傳
  return true;
}

const char* read_status_text() {
  // 例：也可根據錯誤碼切換
  return "OK";
}
// ======================
RosPublisherManager pubman;

void setup() {
  Serial.begin(115200);
  delay(200);
  setup_wifi_fallback();

  if (!pubman.init(NODE_NAME)) {
    Serial.println("初始化失败！");
    return;
  }

  // Int32
  pubman.add_int32_publisher("/battery/mv", 1000, &read_battery_mv);
  // Float32
  pubman.add_float32_publisher("/depth/m", 200, &read_depth_m);
  // Bool
  pubman.add_bool_publisher("/system/autonomous", 1000, &read_is_autonomous);
  // String
  pubman.add_string_publisher("/system/status_text", 1000, &read_status_text);

  Serial.println("准备完毕。");
}

void loop() {
  pubman.spin_some(50);
  // 其餘工作...
}
