#include <Arduino.h>
#include "RosContext.h"
#include "RosPublisherT.h"

// micro-ROS message type
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

RosContext ros;

// 兩個 publisher：一個 Int32、一個 String
RosPublisherT<std_msgs__msg__Int32>  pub_counter;
RosPublisherT<std_msgs__msg__String> pub_status;

static int counter = 0;

void setup() {
  Serial.begin(115200);
  // WiFi / agent 連線請在這裡先處理（你原本的 RosWiFiHelper）

  // 初始化 ROS node 與 executor（一次）
  if (!ros.init("esp32_node")) {
    Serial.println("ROS init failed");
    while(1) delay(1000);
  }

  // 每 200ms 發一個整數
  pub_counter.init(
    ros,
    "telemetry/counter",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    [](std_msgs__msg__Int32& m){
      m.data = counter++;
    },
    200
  );

  // 每 1000ms 發一次狀態字串
  // micro-ROS 的 std_msgs/String 需要記憶體管理，這裡用簡單 helper
  pub_status.init(
    ros,
    "telemetry/status",
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    [](std_msgs__msg__String& m){
      static char buf[64];
      snprintf(buf, sizeof(buf), "uptime=%lu ms", (unsigned long)millis());
      // 指向靜態 buffer（避免動態配置）
      m.data.data = buf;
      m.data.size = strlen(buf);
      m.data.capacity = sizeof(buf);
    },
    1000
  );
}

void loop() {
  // 單一 executor 統一 spin
  ros.spin_some(50);
}
