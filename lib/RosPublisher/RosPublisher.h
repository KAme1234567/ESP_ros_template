#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

template<typename MsgType>
class RosPublisher {
public:
  rcl_node_t node;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rclc_executor_t executor;
  rcl_publisher_t publisher;
  rcl_timer_t timer;
  MsgType msg;

  bool init(const char* node_name, const rosidl_message_type_support_t* ts, const char* topic_name, rcl_timer_callback_t cb, int interval_ms);
  void spin();
};
