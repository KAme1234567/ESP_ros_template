// RosPublisher.cpp
#include "RosPublisher.h"
#include <rclc/executor.h>

bool RosPublisher::init(const char* node_name, const char* topic_name, rcl_timer_callback_t cb, int interval_ms) {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, node_name, "", &support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name) != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(interval_ms), cb) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;

  return true;
}

void RosPublisher::spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}