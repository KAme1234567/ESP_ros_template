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

  return true;
}

template<typename MsgType>
void RosPublisher<MsgType>::spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

// ⚠ 加這個讓模板實作能被鏈結
template class RosPublisher<std_msgs__msg__Int32>;
template class RosPublisher<std_msgs__msg__String>;
