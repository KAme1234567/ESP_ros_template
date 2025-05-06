#include "config.h"
#include "RosWiFiHelper.h"
#include "RosPublisher.h"
#include "rosidl_runtime_c/string_functions.h"
#include "std_msgs/msg/string.h"

RosPublisher<std_msgs__msg__String> publisher;

const int buttonPin1 = 2;
const int buttonPin2 = 4;
int last_state = -1;

char json_buffer[64];

void timer_callback(rcl_timer_t*, int64_t) {
  int b1 = digitalRead(buttonPin1);
  int b2 = digitalRead(buttonPin2);
  int state = (!b1 << 1) | (!b2);

  if (state != last_state) {
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"type\": \"button\", \"value\": \"%d\"}", state);
    Serial.print("[DEBUG] 發送 JSON：");
    Serial.println(json_buffer);

    if (rosidl_runtime_c__String__assign(&publisher.msg.data, json_buffer)) {
      if (rcl_publish(&publisher.publisher, &publisher.msg, NULL) == RCL_RET_OK) {
        Serial.println("[SEND OK]");
      } else {
        Serial.println("[SEND FAIL ❌] rcl_publish failed");
      }
    } else {
      Serial.println("[SEND FAIL ❌] assign failed");
    }

    last_state = state;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  setup_wifi_fallback();

  bool ok = publisher.init(
    "button_pub_node",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    PUB_TOPIC,
    timer_callback,
    1000
  );

  if (ok) {
    Serial.println("✅ ROS Publisher 初始化成功，開始發送...");
  } else {
    Serial.println("❌ ROS Publisher 初始化失敗，請檢查連線與 topic 設定");
  }
}

void loop() {
  publisher.spin();
  delay(100);
}
