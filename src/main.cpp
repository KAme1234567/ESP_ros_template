#include "config.h"
#include "RosWiFiHelper.h"
#include "RosPublisher.h"
#include "std_msgs/msg/string.h"

RosPublisher<std_msgs__msg__String> publisher;

const int potPin = 34;
float last_pot_value = -1.0;

void timer_callback(rcl_timer_t*, int64_t) {
  const int raw_adc = analogRead(potPin);
  const float pot_value = raw_adc / 4095.0f;
  const float delta = fabs(pot_value - last_pot_value);

  // 變化超過 2% 才發送
  if (delta > 0.02f) {
    last_pot_value = pot_value;

    // 轉成 JSON 發送
   0



    Serial.printf("🔄 發送: pot_value = %.2f\n", pot_value);
  } else {
    Serial.println("🔕 無變化，未發送");
  }
}


void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(potPin, INPUT);
  setup_wifi_fallback();

  bool ok = publisher.init(
    "sensor_pub_node",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    PUB_TOPIC,
    timer_callback,
    1000
  );

  if (ok) {
    Serial.println("✅ ROS Publisher 初始化成功，開始發送...");
  } else {
    Serial.println("❌ ROS Publisher 初始化失敗");
  }
}

void loop() {
  publisher.spin();
  delay(100);
}
