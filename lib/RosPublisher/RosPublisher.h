#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <Arduino.h>  // for snprintf, Serial, etc.

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

  // 簡化發送 API（字串、int、float）
  void send(const char* key, const char* value);
  void send(const char* key, int value);
  void send(const char* key, float value);

private:
  char json_buffer[64];  // 避免超過 FastCDR 限制
};
