// RosPublisher.h
#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>

class RosPublisher {
public:
  rcl_node_t node;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rclc_executor_t executor;
  rcl_publisher_t publisher;
  rcl_timer_t timer;
  std_msgs__msg__Int32 msg;

  bool init(const char* node_name, const char* topic_name, rcl_timer_callback_t cb, int interval_ms);
  void spin();
};