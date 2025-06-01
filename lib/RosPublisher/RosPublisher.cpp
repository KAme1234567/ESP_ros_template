#include "RosPublisher.h"
#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/string.h"

template<typename MsgType>

bool RosPublisher<MsgType>::init(const char* node_name, const rosidl_message_type_support_t* ts, const char* topic_name, rcl_timer_callback_t cb, int interval_ms) {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, node_name, "", &support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&publisher, &node, ts, topic_name) != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(interval_ms), cb) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;

  // 初始化 msg.data（針對 String 類型）
  rosidl_runtime_c__String__init(&msg.data);
  return true;
}

template<typename MsgType>
void RosPublisher<MsgType>::spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

// 傳送 const char*（字串形式）
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, const char* value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":\"%s\"}", key, value);

  if (strlen(json_buffer) >= sizeof(json_buffer) - 1) {
    Serial.println("❌ JSON 太長，未送出");
    return;
  }

  if (!rosidl_runtime_c__String__assign(&msg.data, json_buffer)) {
    Serial.println("❌ String assign 失敗");
    return;
  }

  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.print("❌ Publish 失敗: ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
  } else {
    Serial.println("[SEND OK]");
  }
}

// 傳送 int
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, int value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":%d}", key, value);
  send(key, json_buffer);  // 重用字串版
}

// 傳送 float
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, float value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":%.2f}", key, value);
  send(key, json_buffer);  // 重用字串版
}

// 強制實例化模板
template class RosPublisher<std_msgs__msg__String>;
