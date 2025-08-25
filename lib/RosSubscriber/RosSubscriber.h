#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

// ===== 每種型別最多訂閱數（可依需求調整） =====
#ifndef ROSS_MAX_INT32
#define ROSS_MAX_INT32 8
#endif

#ifndef ROSS_MAX_FLOAT32
#define ROSS_MAX_FLOAT32 8
#endif

#ifndef ROSS_MAX_BOOL
#define ROSS_MAX_BOOL 8
#endif

#ifndef ROSS_MAX_STRING
#define ROSS_MAX_STRING 8
#endif

#ifndef ROSS_STRING_PREALLOC
#define ROSS_STRING_PREALLOC 256   // 預先配置 256 bytes（可改）
#endif

// ===== 使用者回呼型別（含 user_data） =====
typedef void (*Int32SubCb)(const std_msgs__msg__Int32* , void* user_data);
typedef void (*Float32SubCb)(const std_msgs__msg__Float32*, void* user_data);
typedef void (*BoolSubCb)(const std_msgs__msg__Bool* , void* user_data);
typedef void (*StringSubCb)(const std_msgs__msg__String*, void* user_data);

class RosSubscriberManager {
public:

  RosSubscriberManager();
  ~RosSubscriberManager();

  // 初始化（node_name 建議從 config.h 帶入）
  bool init(const char* node_name);

  // 清理（也會在析構時自動呼叫）
  void fini();

  // 新增各型別訂閱（回傳 >=0 表成功的 slot index；-1 表失敗/已滿）
  int add_int32_subscription (const char* topic_name, Int32SubCb  cb, void* user_data = nullptr,
                              rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_float32_subscription(const char* topic_name, Float32SubCb cb, void* user_data = nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_bool_subscription  (const char* topic_name, BoolSubCb   cb, void* user_data = nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_string_subscription(const char* topic_name, StringSubCb cb, void* user_data = nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);

  // 在 loop 中呼叫
  void spin_some(uint32_t timeout_ms = 50);

private:
  // 靜態 adapter，把 executor 的 (const void*) msgin 轉派回實例
  static RosSubscriberManager* s_instance;
  static void int32_callback_adapter (const void* msgin);
  static void float32_callback_adapter(const void* msgin);
  static void bool_callback_adapter   (const void* msgin);
  static void string_callback_adapter (const void* msgin);

  // 實例端查找對應 slot 並呼叫使用者回呼
  void on_int32_msg (const void* msgin);
  void on_float32_msg(const void* msgin);
  void on_bool_msg   (const void* msgin);
  void on_string_msg (const void* msgin);

  // micro-ROS core
  rcl_allocator_t  allocator_;
  rclc_support_t   support_;
  rcl_node_t       node_;
  rclc_executor_t  executor_;
  bool             inited_;

  // Int32
  struct Int32Slot {
    bool used;
    rcl_subscription_t sub;
    std_msgs__msg__Int32 msg;
    Int32SubCb cb;
    void* user;
    const char* topic;
  } int32_slots_[ROSS_MAX_INT32];

  // Float32
  struct Float32Slot {
    bool used;
    rcl_subscription_t sub;
    std_msgs__msg__Float32 msg;
    Float32SubCb cb;
    void* user;
    const char* topic;
  } float32_slots_[ROSS_MAX_FLOAT32];

  // Bool
  struct BoolSlot {
    bool used;
    rcl_subscription_t sub;
    std_msgs__msg__Bool msg;
    BoolSubCb cb;
    void* user;
    const char* topic;
  } bool_slots_[ROSS_MAX_BOOL];

  // String
  struct StringSlot {
    bool used;
    rcl_subscription_t sub;
    std_msgs__msg__String msg;
    StringSubCb cb;
    void* user;
    const char* topic;
  } string_slots_[ROSS_MAX_STRING];

  // helpers
  int find_free_int32 ();
  int find_free_float32();
  int find_free_bool  ();
  int find_free_string();
};
