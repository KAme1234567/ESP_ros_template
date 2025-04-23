// RosSubscriber.cpp
#include "RosSubscriber.h"
#include <rclc/executor.h>


bool RosSubscriber::init(const char* node_name, const char* topic_name, rclc_subscription_callback_t cb) {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, node_name, "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, cb, ON_NEW_DATA) != RCL_RET_OK) return false;

  return true;
}

void RosSubscriber::spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
