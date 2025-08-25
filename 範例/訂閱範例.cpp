#include <Arduino.h>
#include "config.h"
#include <RosSubscriber.h>
#include "RosWiFiHelper.h"

// ===== 訂閱回呼們 =====
void on_mode_int(const std_msgs__msg__Int32* msg, void* user) {
  (void)user;
  Serial.print("[/cmd/mode] Int32 = "); Serial.println((int)msg->data);
}

void on_target_depth(const std_msgs__msg__Float32* msg, void* user) {
  (void)user;
  Serial.print("[/cmd/target_depth] Float32 = "); Serial.println(msg->data, 3);
}

void on_arm_state(const std_msgs__msg__Bool* msg, void* user) {
  (void)user;
  Serial.print("[/cmd/armed] Bool = ");
  Serial.println(msg->data ? "true" : "false");
}

void on_note_text(const std_msgs__msg__String* msg, void* user) {
  (void)user;
  // rosidl string 以 null 結尾，安全列印
  Serial.print("[/cmd/note] String = ");
  Serial.println(msg->data.data ? msg->data.data : "");
}

// ===== Manager =====
RosSubscriberManager subman;

void setup() {
  Serial.begin(115200);
  delay(200);

  setup_wifi_fallback();          // 你的 WiFi 連線/切換

  if (!subman.init(NODE_NAME)) {
    Serial.println("初始化失败!");
    return;
  }

  // 訂閱四種型別
  subman.add_int32_subscription ("/cmd/mode",         &on_mode_int);
  subman.add_float32_subscription("/cmd/target_depth",&on_target_depth);
  subman.add_bool_subscription  ("/cmd/armed",        &on_arm_state);
  subman.add_string_subscription("/cmd/note",         &on_note_text);

  Serial.println("准备好了.");
}

void loop() {
  subman.spin_some(50);
  // 其他任務...
}
