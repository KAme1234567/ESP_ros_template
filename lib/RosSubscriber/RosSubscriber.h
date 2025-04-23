// RosSubscriber.h
#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

class RosSubscriber {
public:
  rcl_node_t node;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rclc_executor_t executor;
  rcl_subscription_t subscriber;
  std_msgs__msg__Int32 msg;

  bool init(const char* node_name, const char* topic_name, rclc_subscription_callback_t cb);
  void spin();
};
