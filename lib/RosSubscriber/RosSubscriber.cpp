#include "RosSubscriber.h"
#include <string.h>

RosSubscriberManager* RosSubscriberManager::s_instance = nullptr;

RosSubscriberManager::RosSubscriberManager()
: allocator_(rcl_get_default_allocator()),
  inited_(false)
{
  memset(&support_,  0, sizeof(support_));
  memset(&node_,     0, sizeof(node_));
  memset(&executor_, 0, sizeof(executor_));

  for (int i=0;i<ROSS_MAX_INT32;i++){
    int32_slots_[i].used=false;
    memset(&int32_slots_[i].sub,0,sizeof(rcl_subscription_t));
    std_msgs__msg__Int32__init(&int32_slots_[i].msg);
    int32_slots_[i].cb=nullptr;
    int32_slots_[i].user=nullptr;
    int32_slots_[i].topic=nullptr;
  }
  for (int i=0;i<ROSS_MAX_FLOAT32;i++){
    float32_slots_[i].used=false;
    memset(&float32_slots_[i].sub,0,sizeof(rcl_subscription_t));
    std_msgs__msg__Float32__init(&float32_slots_[i].msg);
    float32_slots_[i].cb=nullptr;
    float32_slots_[i].user=nullptr;
    float32_slots_[i].topic=nullptr;
  }
  for (int i=0;i<ROSS_MAX_BOOL;i++){
    bool_slots_[i].used=false;
    memset(&bool_slots_[i].sub,0,sizeof(rcl_subscription_t));
    std_msgs__msg__Bool__init(&bool_slots_[i].msg);
    bool_slots_[i].cb=nullptr;
    bool_slots_[i].user=nullptr;
    bool_slots_[i].topic=nullptr;
  }
for (int i=0;i<ROSS_MAX_STRING;i++){
  string_slots_[i].used=false;
  memset(&string_slots_[i].sub,0,sizeof(rcl_subscription_t));
  std_msgs__msg__String__init(&string_slots_[i].msg);
  // ★ 預先配置容量（size=0, capacity=ROSS_STRING_PREALLOC+1）
  rosidl_runtime_c__String__assignn(&string_slots_[i].msg.data, "", ROSS_STRING_PREALLOC);
  string_slots_[i].cb=nullptr;
  string_slots_[i].user=nullptr;
  string_slots_[i].topic=nullptr;
}
}

RosSubscriberManager::~RosSubscriberManager() {
  fini();
}

bool RosSubscriberManager::init(const char* node_name) {
  if (inited_) return true;

  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, node_name, "", &support_) != RCL_RET_OK) return false;

  const size_t max_handles = (ROSS_MAX_INT32 + ROSS_MAX_FLOAT32 + ROSS_MAX_BOOL + ROSS_MAX_STRING);
  if (rclc_executor_init(&executor_, &support_.context, max_handles, &allocator_) != RCL_RET_OK) return false;

  s_instance = this;
  inited_ = true;
  return true;
}

void RosSubscriberManager::fini() {
  if (!inited_) return;

  for (int i=0;i<ROSS_MAX_INT32;i++){
    if (int32_slots_[i].used){
      rcl_subscription_fini(&int32_slots_[i].sub, &node_);
      int32_slots_[i].used=false;
    }
    std_msgs__msg__Int32__fini(&int32_slots_[i].msg);
  }
  for (int i=0;i<ROSS_MAX_FLOAT32;i++){
    if (float32_slots_[i].used){
      rcl_subscription_fini(&float32_slots_[i].sub, &node_);
      float32_slots_[i].used=false;
    }
    std_msgs__msg__Float32__fini(&float32_slots_[i].msg);
  }
  for (int i=0;i<ROSS_MAX_BOOL;i++){
    if (bool_slots_[i].used){
      rcl_subscription_fini(&bool_slots_[i].sub, &node_);
      bool_slots_[i].used=false;
    }
    std_msgs__msg__Bool__fini(&bool_slots_[i].msg);
  }
  for (int i=0;i<ROSS_MAX_STRING;i++){
    if (string_slots_[i].used){
      rcl_subscription_fini(&string_slots_[i].sub, &node_);
      string_slots_[i].used=false;
    }
    std_msgs__msg__String__fini(&string_slots_[i].msg);
  }

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
  inited_ = false;
  s_instance = nullptr;
}

int RosSubscriberManager::find_free_int32 () { for (int i=0;i<ROSS_MAX_INT32 ;++i) if (!int32_slots_[i].used)  return i; return -1; }
int RosSubscriberManager::find_free_float32(){ for (int i=0;i<ROSS_MAX_FLOAT32;++i) if (!float32_slots_[i].used) return i; return -1; }
int RosSubscriberManager::find_free_bool  () { for (int i=0;i<ROSS_MAX_BOOL  ;++i) if (!bool_slots_[i].used)   return i; return -1; }
int RosSubscriberManager::find_free_string(){ for (int i=0;i<ROSS_MAX_STRING;++i) if (!string_slots_[i].used) return i; return -1; }

int RosSubscriberManager::add_int32_subscription(const char* topic_name, Int32SubCb cb, void* user_data,
                                                 rclc_executor_handle_invocation_t when) {
  if (!inited_ || cb == nullptr) return -1;
  int idx = find_free_int32(); if (idx < 0) return -1;
  auto& S = int32_slots_[idx];

  if (rclc_subscription_init_default(&S.sub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name) != RCL_RET_OK) return -1;

  S.cb   = cb;
  S.user = user_data;
  S.topic= topic_name;
  S.used = true;

  if (rclc_executor_add_subscription(&executor_, &S.sub, &S.msg,
        &RosSubscriberManager::int32_callback_adapter, when) != RCL_RET_OK) {
    S.used=false;
    return -1;
  }
  return idx;
}

int RosSubscriberManager::add_float32_subscription(const char* topic_name, Float32SubCb cb, void* user_data,
                                                   rclc_executor_handle_invocation_t when) {
  if (!inited_ || cb == nullptr) return -1;
  int idx = find_free_float32(); if (idx < 0) return -1;
  auto& S = float32_slots_[idx];

  if (rclc_subscription_init_default(&S.sub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), topic_name) != RCL_RET_OK) return -1;

  S.cb   = cb;
  S.user = user_data;
  S.topic= topic_name;
  S.used = true;

  if (rclc_executor_add_subscription(&executor_, &S.sub, &S.msg,
        &RosSubscriberManager::float32_callback_adapter, when) != RCL_RET_OK) {
    S.used=false;
    return -1;
  }
  return idx;
}

int RosSubscriberManager::add_bool_subscription(const char* topic_name, BoolSubCb cb, void* user_data,
                                                rclc_executor_handle_invocation_t when) {
  if (!inited_ || cb == nullptr) return -1;
  int idx = find_free_bool(); if (idx < 0) return -1;
  auto& S = bool_slots_[idx];

  if (rclc_subscription_init_default(&S.sub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), topic_name) != RCL_RET_OK) return -1;

  S.cb   = cb;
  S.user = user_data;
  S.topic= topic_name;
  S.used = true;

  if (rclc_executor_add_subscription(&executor_, &S.sub, &S.msg,
        &RosSubscriberManager::bool_callback_adapter, when) != RCL_RET_OK) {
    S.used=false;
    return -1;
  }
  return idx;
}

int RosSubscriberManager::add_string_subscription(const char* topic_name, StringSubCb cb, void* user_data,
                                                  rclc_executor_handle_invocation_t when) {
  if (!inited_ || cb == nullptr) return -1;
  int idx = find_free_string(); if (idx < 0) return -1;
  auto& S = string_slots_[idx];

  if (rclc_subscription_init_default(&S.sub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name) != RCL_RET_OK) return -1;

  S.cb   = cb;
  S.user = user_data;
  S.topic= topic_name;
  S.used = true;

  if (rclc_executor_add_subscription(&executor_, &S.sub, &S.msg,
        &RosSubscriberManager::string_callback_adapter, when) != RCL_RET_OK) {
    S.used=false;
    return -1;
  }
  return idx;
}

void RosSubscriberManager::spin_some(uint32_t timeout_ms) {
  if (!inited_) return;
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(timeout_ms));
}

// ===== 靜態 adapter → 實例處理 =====
void RosSubscriberManager::int32_callback_adapter (const void* msgin){ if (s_instance) s_instance->on_int32_msg (msgin); }
void RosSubscriberManager::float32_callback_adapter(const void* msgin){ if (s_instance) s_instance->on_float32_msg(msgin); }
void RosSubscriberManager::bool_callback_adapter   (const void* msgin){ if (s_instance) s_instance->on_bool_msg   (msgin); }
void RosSubscriberManager::string_callback_adapter (const void* msgin){ if (s_instance) s_instance->on_string_msg (msgin); }

// ===== 依 msg 指標比對 slot 並呼叫使用者回呼 =====
void RosSubscriberManager::on_int32_msg(const void* msgin) {
  const std_msgs__msg__Int32* M = (const std_msgs__msg__Int32*)msgin;
  for (int i=0;i<ROSS_MAX_INT32;i++){
    auto& S = int32_slots_[i];
    if (S.used && (&S.msg == M)) { if (S.cb) S.cb(M, S.user); return; }
  }
}

void RosSubscriberManager::on_float32_msg(const void* msgin) {
  const std_msgs__msg__Float32* M = (const std_msgs__msg__Float32*)msgin;
  for (int i=0;i<ROSS_MAX_FLOAT32;i++){
    auto& S = float32_slots_[i];
    if (S.used && (&S.msg == M)) { if (S.cb) S.cb(M, S.user); return; }
  }
}

void RosSubscriberManager::on_bool_msg(const void* msgin) {
  const std_msgs__msg__Bool* M = (const std_msgs__msg__Bool*)msgin;
  for (int i=0;i<ROSS_MAX_BOOL;i++){
    auto& S = bool_slots_[i];
    if (S.used && (&S.msg == M)) { if (S.cb) S.cb(M, S.user); return; }
  }
}

void RosSubscriberManager::on_string_msg(const void* msgin) {
  const std_msgs__msg__String* M = (const std_msgs__msg__String*)msgin;
  for (int i=0;i<ROSS_MAX_STRING;i++){
    auto& S = string_slots_[i];
    if (S.used && (&S.msg == M)) { if (S.cb) S.cb(M, S.user); return; }
  }
}
