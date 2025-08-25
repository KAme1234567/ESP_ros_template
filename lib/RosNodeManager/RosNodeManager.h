#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

// ===== 可調上限（依需求調整）=====
// Publishers
#ifndef ROSN_MAX_INT32_PUB
#define ROSN_MAX_INT32_PUB 8
#endif
#ifndef ROSN_MAX_FLOAT32_PUB
#define ROSN_MAX_FLOAT32_PUB 8
#endif
#ifndef ROSN_MAX_BOOL_PUB
#define ROSN_MAX_BOOL_PUB 8
#endif
#ifndef ROSN_MAX_STRING_PUB
#define ROSN_MAX_STRING_PUB 8
#endif
// Subscribers
#ifndef ROSN_MAX_INT32_SUB
#define ROSN_MAX_INT32_SUB 8
#endif
#ifndef ROSN_MAX_FLOAT32_SUB
#define ROSN_MAX_FLOAT32_SUB 8
#endif
#ifndef ROSN_MAX_BOOL_SUB
#define ROSN_MAX_BOOL_SUB 8
#endif
#ifndef ROSN_MAX_STRING_SUB
#define ROSN_MAX_STRING_SUB 8
#endif

// 預先配置每個 String 訂閱訊息的容量（避免第一次動態配置/丟包）
#ifndef ROSN_STRING_PREALLOC
#define ROSN_STRING_PREALLOC 256
#endif

// ===== Publisher providers（無 user_data 版）=====
typedef int32_t     (*Int32ProviderFn)();
typedef float       (*Float32ProviderFn)();
typedef bool        (*BoolProviderFn)();
typedef const char* (*StringProviderFn)();

// ===== Subscriber callbacks（含 user_data）=====
typedef void (*Int32SubCb)( const std_msgs__msg__Int32*  , void* user_data );
typedef void (*Float32SubCb)(const std_msgs__msg__Float32*, void* user_data );
typedef void (*BoolSubCb)(   const std_msgs__msg__Bool*   , void* user_data );
typedef void (*StringSubCb)( const std_msgs__msg__String* , void* user_data );

class RosNodeManager {
public:
  RosNodeManager();
  ~RosNodeManager();

  bool init(const char* node_name);
  void fini();

  void spin_some(uint32_t timeout_ms = 50);

  // === Add Publishers ===
  int add_int32_publisher (const char* topic_name, uint32_t period_ms, Int32ProviderFn  provider);
  int add_float32_publisher(const char* topic_name, uint32_t period_ms, Float32ProviderFn provider);
  int add_bool_publisher   (const char* topic_name, uint32_t period_ms, BoolProviderFn   provider);
  int add_string_publisher (const char* topic_name, uint32_t period_ms, StringProviderFn provider);

  // === Add Subscriptions ===
  int add_int32_subscription (const char* topic_name, Int32SubCb  cb, void* user_data=nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_float32_subscription(const char* topic_name, Float32SubCb cb, void* user_data=nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_bool_subscription   (const char* topic_name, BoolSubCb   cb, void* user_data=nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);
  int add_string_subscription (const char* topic_name, StringSubCb cb, void* user_data=nullptr,
                               rclc_executor_handle_invocation_t when = ON_NEW_DATA);

private:
  // === Static adapters ===
  static RosNodeManager* s_instance;
  static void timer_adapter(rcl_timer_t* timer, int64_t last_call_time);

  static void int32_sub_adapter (const void* msgin);
  static void float32_sub_adapter(const void* msgin);
  static void bool_sub_adapter   (const void* msgin);
  static void string_sub_adapter (const void* msgin);

  // === Instance handlers ===
  void on_timer(rcl_timer_t* timer);

  void on_int32_msg (const void* msgin);
  void on_float32_msg(const void* msgin);
  void on_bool_msg   (const void* msgin);
  void on_string_msg (const void* msgin);

  // === Core ===
  rcl_allocator_t  allocator_;
  rclc_support_t   support_;
  rcl_node_t       node_;
  rclc_executor_t  executor_;
  bool             inited_;

  // === Pub slots ===
  struct Int32Pub {
    bool used; rcl_publisher_t pub; rcl_timer_t timer;
    std_msgs__msg__Int32 msg; Int32ProviderFn provider; const char* topic;
  } int32_pubs_[ROSN_MAX_INT32_PUB];

  struct Float32Pub {
    bool used; rcl_publisher_t pub; rcl_timer_t timer;
    std_msgs__msg__Float32 msg; Float32ProviderFn provider; const char* topic;
  } float32_pubs_[ROSN_MAX_FLOAT32_PUB];

  struct BoolPub {
    bool used; rcl_publisher_t pub; rcl_timer_t timer;
    std_msgs__msg__Bool msg; BoolProviderFn provider; const char* topic;
  } bool_pubs_[ROSN_MAX_BOOL_PUB];

  struct StringPub {
    bool used; rcl_publisher_t pub; rcl_timer_t timer;
    std_msgs__msg__String msg; bool msg_inited;
    StringProviderFn provider; const char* topic;
  } string_pubs_[ROSN_MAX_STRING_PUB];

  // === Sub slots ===
  struct Int32Sub {
    bool used; rcl_subscription_t sub;
    std_msgs__msg__Int32 msg; Int32SubCb cb; void* user; const char* topic;
  } int32_subs_[ROSN_MAX_INT32_SUB];

  struct Float32Sub {
    bool used; rcl_subscription_t sub;
    std_msgs__msg__Float32 msg; Float32SubCb cb; void* user; const char* topic;
  } float32_subs_[ROSN_MAX_FLOAT32_SUB];

  struct BoolSub {
    bool used; rcl_subscription_t sub;
    std_msgs__msg__Bool msg; BoolSubCb cb; void* user; const char* topic;
  } bool_subs_[ROSN_MAX_BOOL_SUB];

  struct StringSub {
    bool used; rcl_subscription_t sub;
    std_msgs__msg__String msg; bool msg_inited;
    StringSubCb cb; void* user; const char* topic;
  } string_subs_[ROSN_MAX_STRING_SUB];

  // helpers
  int free_int32_pub (); int free_float32_pub(); int free_bool_pub (); int free_string_pub();
  int free_int32_sub (); int free_float32_sub(); int free_bool_sub (); int free_string_sub();
};
