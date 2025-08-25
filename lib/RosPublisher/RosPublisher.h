#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <Arduino.h>  // for snprintf, Serial, etc.

// ===== msg types =====
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
// string helpers
#include <rosidl_runtime_c/string_functions.h>

<<<<<<< HEAD
// ===== per-type max counts (可依需求調整) =====
#ifndef ROSP_MAX_INT32
#define ROSP_MAX_INT32 8
#endif

#ifndef ROSP_MAX_FLOAT32
#define ROSP_MAX_FLOAT32 8
#endif

#ifndef ROSP_MAX_BOOL
#define ROSP_MAX_BOOL 8
#endif

#ifndef ROSP_MAX_STRING
#define ROSP_MAX_STRING 8
#endif

// ===== provider 函式型態 =====
typedef int32_t     (*Int32ProviderFn)();
typedef float       (*Float32ProviderFn)();
typedef bool        (*BoolProviderFn)();
typedef const char* (*StringProviderFn)();

class RosPublisherManager {
public:

  RosPublisherManager();
  ~RosPublisherManager();

  // 初始化（node_name 建議從 config.h 帶入）
  bool init(const char* node_name);

  // 終結（釋放資源；可不呼叫，析構時也會做）
  void fini();

  // 新增各型別 publisher（回傳 >=0 表示成功的 slot index；-1 失敗/滿了）
  int add_int32_publisher (const char* topic_name, uint32_t period_ms, Int32ProviderFn provider);
  int add_float32_publisher(const char* topic_name, uint32_t period_ms, Float32ProviderFn provider);
  int add_bool_publisher  (const char* topic_name, uint32_t period_ms, BoolProviderFn provider);
  int add_string_publisher(const char* topic_name, uint32_t period_ms, StringProviderFn provider);

  // 在 loop 中呼叫
  void spin_some(uint32_t timeout_ms = 50);

private:
  // static → 將 timer callback 導到實例方法
  static RosPublisherManager* s_instance;
  static void timer_callback_adapter(rcl_timer_t* timer, int64_t last_call_time);
  void on_timer(rcl_timer_t* timer);

  // micro-ROS core
  rcl_allocator_t  allocator_;
  rclc_support_t   support_;
  rcl_node_t       node_;
  rclc_executor_t  executor_;
  bool             inited_;

  // ——— Int32 slots ———
  struct Int32Slot {
    bool used;
    rcl_publisher_t pub;
    rcl_timer_t     timer;
    std_msgs__msg__Int32 msg;
    Int32ProviderFn provider;
    const char* topic;
  } int32_slots_[ROSP_MAX_INT32];

  // ——— Float32 slots ———
  struct Float32Slot {
    bool used;
    rcl_publisher_t pub;
    rcl_timer_t     timer;
    std_msgs__msg__Float32 msg;
    Float32ProviderFn provider;
    const char* topic;
  } float32_slots_[ROSP_MAX_FLOAT32];

  // ——— Bool slots ———
  struct BoolSlot {
    bool used;
    rcl_publisher_t pub;
    rcl_timer_t     timer;
    std_msgs__msg__Bool msg;
    BoolProviderFn provider;
    const char* topic;
  } bool_slots_[ROSP_MAX_BOOL];

  // ——— String slots ———
  struct StringSlot {
    bool used;
    rcl_publisher_t pub;
    rcl_timer_t     timer;
    std_msgs__msg__String msg;
    StringProviderFn provider;
    const char* topic;
  } string_slots_[ROSP_MAX_STRING];

  // helpers
  int find_free_int32 ();
  int find_free_float32();
  int find_free_bool  ();
  int find_free_string();
=======
  bool init(const char* node_name, const rosidl_message_type_support_t* ts, const char* topic_name, rcl_timer_callback_t cb, int interval_ms);
  void spin();

  // 簡化發送 API（字串、int、float）
  void send(const char* key, const char* value);
  void send(const char* key, int value);
  void send(const char* key, float value);

private:
  char json_buffer[64];  // 避免超過 FastCDR 限制
>>>>>>> 30acb290d12f5dac78a29d347b8c70b6f273b1af
};
