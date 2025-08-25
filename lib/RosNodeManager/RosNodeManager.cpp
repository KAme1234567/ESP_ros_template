#include "RosNodeManager.h"
#include <string.h>

RosNodeManager* RosNodeManager::s_instance = nullptr;

RosNodeManager::RosNodeManager()
: allocator_(rcl_get_default_allocator()), inited_(false)
{
  memset(&support_,0,sizeof(support_));
  memset(&node_,0,sizeof(node_));
  memset(&executor_,0,sizeof(executor_));

  // pubs
  for (int i=0;i<ROSN_MAX_INT32_PUB;i++){ auto& S=int32_pubs_[i];
    S.used=false; memset(&S.pub,0,sizeof(S.pub)); memset(&S.timer,0,sizeof(S.timer));
    S.msg.data=0; S.provider=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_FLOAT32_PUB;i++){ auto& S=float32_pubs_[i];
    S.used=false; memset(&S.pub,0,sizeof(S.pub)); memset(&S.timer,0,sizeof(S.timer));
    S.msg.data=0.0f; S.provider=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_BOOL_PUB;i++){ auto& S=bool_pubs_[i];
    S.used=false; memset(&S.pub,0,sizeof(S.pub)); memset(&S.timer,0,sizeof(S.timer));
    S.msg.data=false; S.provider=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_STRING_PUB;i++){ auto& S=string_pubs_[i];
    S.used=false; memset(&S.pub,0,sizeof(S.pub)); memset(&S.timer,0,sizeof(S.timer));
    memset(&S.msg,0,sizeof(S.msg)); S.msg_inited=false; S.provider=nullptr; S.topic=nullptr; }

  // subs
  for (int i=0;i<ROSN_MAX_INT32_SUB;i++){ auto& S=int32_subs_[i];
    S.used=false; memset(&S.sub,0,sizeof(S.sub)); S.msg.data=0; S.cb=nullptr; S.user=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_FLOAT32_SUB;i++){ auto& S=float32_subs_[i];
    S.used=false; memset(&S.sub,0,sizeof(S.sub)); S.msg.data=0.0f; S.cb=nullptr; S.user=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_BOOL_SUB;i++){ auto& S=bool_subs_[i];
    S.used=false; memset(&S.sub,0,sizeof(S.sub)); S.msg.data=false; S.cb=nullptr; S.user=nullptr; S.topic=nullptr; }
  for (int i=0;i<ROSN_MAX_STRING_SUB;i++){ auto& S=string_subs_[i];
    S.used=false; memset(&S.sub,0,sizeof(S.sub)); memset(&S.msg,0,sizeof(S.msg));
    S.msg_inited=false; S.cb=nullptr; S.user=nullptr; S.topic=nullptr; }
}

RosNodeManager::~RosNodeManager(){ fini(); }

bool RosNodeManager::init(const char* node_name){
  if (inited_) return true;
  if (rclc_support_init(&support_,0,NULL,&allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, node_name, "", &support_) != RCL_RET_OK) return false;

  const size_t max_handles =
      (ROSN_MAX_INT32_PUB + ROSN_MAX_FLOAT32_PUB + ROSN_MAX_BOOL_PUB + ROSN_MAX_STRING_PUB) // timers
    + (ROSN_MAX_INT32_SUB + ROSN_MAX_FLOAT32_SUB + ROSN_MAX_BOOL_SUB + ROSN_MAX_STRING_SUB); // subs

  if (rclc_executor_init(&executor_, &support_.context, max_handles, &allocator_) != RCL_RET_OK) return false;

  s_instance = this;
  inited_ = true;
  return true;
}

void RosNodeManager::fini(){
  if (!inited_) return;

  // pubs
  for (int i=0;i<ROSN_MAX_INT32_PUB;i++) if (int32_pubs_[i].used){
    rcl_timer_fini(&int32_pubs_[i].timer); rcl_publisher_fini(&int32_pubs_[i].pub, &node_); int32_pubs_[i].used=false; }
  for (int i=0;i<ROSN_MAX_FLOAT32_PUB;i++) if (float32_pubs_[i].used){
    rcl_timer_fini(&float32_pubs_[i].timer); rcl_publisher_fini(&float32_pubs_[i].pub, &node_); float32_pubs_[i].used=false; }
  for (int i=0;i<ROSN_MAX_BOOL_PUB;i++) if (bool_pubs_[i].used){
    rcl_timer_fini(&bool_pubs_[i].timer); rcl_publisher_fini(&bool_pubs_[i].pub, &node_); bool_pubs_[i].used=false; }
  for (int i=0;i<ROSN_MAX_STRING_PUB;i++){ auto& S=string_pubs_[i];
    if (S.used){ rcl_timer_fini(&S.timer); rcl_publisher_fini(&S.pub, &node_); S.used=false; }
    if (S.msg_inited){ std_msgs__msg__String__fini(&S.msg); S.msg_inited=false; }
  }

  // subs
  for (int i=0;i<ROSN_MAX_INT32_SUB;i++){ if (int32_subs_[i].used){
      rcl_subscription_fini(&int32_subs_[i].sub, &node_); int32_subs_[i].used=false; } }
  for (int i=0;i<ROSN_MAX_FLOAT32_SUB;i++){ if (float32_subs_[i].used){
      rcl_subscription_fini(&float32_subs_[i].sub, &node_); float32_subs_[i].used=false; } }
  for (int i=0;i<ROSN_MAX_BOOL_SUB;i++){ if (bool_subs_[i].used){
      rcl_subscription_fini(&bool_subs_[i].sub, &node_); bool_subs_[i].used=false; } }
  for (int i=0;i<ROSN_MAX_STRING_SUB;i++){ auto& S=string_subs_[i];
    if (S.used){ rcl_subscription_fini(&S.sub, &node_); S.used=false; }
    if (S.msg_inited){ std_msgs__msg__String__fini(&S.msg); S.msg_inited=false; }
  }

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
  inited_ = false;
  s_instance = nullptr;
}

void RosNodeManager::spin_some(uint32_t timeout_ms){
  if (!inited_) return;
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(timeout_ms));
}

// ===== helpers =====
int RosNodeManager::free_int32_pub(){ for(int i=0;i<ROSN_MAX_INT32_PUB;i++) if(!int32_pubs_[i].used) return i; return -1; }
int RosNodeManager::free_float32_pub(){ for(int i=0;i<ROSN_MAX_FLOAT32_PUB;i++) if(!float32_pubs_[i].used) return i; return -1; }
int RosNodeManager::free_bool_pub(){ for(int i=0;i<ROSN_MAX_BOOL_PUB;i++) if(!bool_pubs_[i].used) return i; return -1; }
int RosNodeManager::free_string_pub(){ for(int i=0;i<ROSN_MAX_STRING_PUB;i++) if(!string_pubs_[i].used) return i; return -1; }

int RosNodeManager::free_int32_sub(){ for(int i=0;i<ROSN_MAX_INT32_SUB;i++) if(!int32_subs_[i].used) return i; return -1; }
int RosNodeManager::free_float32_sub(){ for(int i=0;i<ROSN_MAX_FLOAT32_SUB;i++) if(!float32_subs_[i].used) return i; return -1; }
int RosNodeManager::free_bool_sub(){ for(int i=0;i<ROSN_MAX_BOOL_SUB;i++) if(!bool_subs_[i].used) return i; return -1; }
int RosNodeManager::free_string_sub(){ for(int i=0;i<ROSN_MAX_STRING_SUB;i++) if(!string_subs_[i].used) return i; return -1; }

// ===== add publishers =====
int RosNodeManager::add_int32_publisher(const char* topic, uint32_t period_ms, Int32ProviderFn provider){
  if(!inited_ || !provider) return -1;
  int idx=free_int32_pub(); if(idx<0) return -1;
  auto& S=int32_pubs_[idx];
  if(rclc_publisher_init_default(&S.pub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32), topic)!=RCL_RET_OK) return -1;
  if(rclc_timer_init_default(&S.timer,&support_, RCL_MS_TO_NS(period_ms), &RosNodeManager::timer_adapter)!=RCL_RET_OK) return -1;
  S.msg.data=0; S.provider=provider; S.topic=topic; S.used=true;
  if(rclc_executor_add_timer(&executor_, &S.timer)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_float32_publisher(const char* topic, uint32_t period_ms, Float32ProviderFn provider){
  if(!inited_ || !provider) return -1;
  int idx=free_float32_pub(); if(idx<0) return -1;
  auto& S=float32_pubs_[idx];
  if(rclc_publisher_init_default(&S.pub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32), topic)!=RCL_RET_OK) return -1;
  if(rclc_timer_init_default(&S.timer,&support_, RCL_MS_TO_NS(period_ms), &RosNodeManager::timer_adapter)!=RCL_RET_OK) return -1;
  S.msg.data=0.0f; S.provider=provider; S.topic=topic; S.used=true;
  if(rclc_executor_add_timer(&executor_, &S.timer)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_bool_publisher(const char* topic, uint32_t period_ms, BoolProviderFn provider){
  if(!inited_ || !provider) return -1;
  int idx=free_bool_pub(); if(idx<0) return -1;
  auto& S=bool_pubs_[idx];
  if(rclc_publisher_init_default(&S.pub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool), topic)!=RCL_RET_OK) return -1;
  if(rclc_timer_init_default(&S.timer,&support_, RCL_MS_TO_NS(period_ms), &RosNodeManager::timer_adapter)!=RCL_RET_OK) return -1;
  S.msg.data=false; S.provider=provider; S.topic=topic; S.used=true;
  if(rclc_executor_add_timer(&executor_, &S.timer)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_string_publisher(const char* topic, uint32_t period_ms, StringProviderFn provider){
  if(!inited_ || !provider) return -1;
  int idx=free_string_pub(); if(idx<0) return -1;
  auto& S=string_pubs_[idx];
  if(rclc_publisher_init_default(&S.pub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,String), topic)!=RCL_RET_OK) return -1;
  if(rclc_timer_init_default(&S.timer,&support_, RCL_MS_TO_NS(period_ms), &RosNodeManager::timer_adapter)!=RCL_RET_OK) return -1;

  // ✅ 在此時才初始化 String 訊息
  if(!std_msgs__msg__String__init(&S.msg)) return -1;
  S.msg_inited = true;

  S.provider=provider; S.topic=topic; S.used=true;
  if(rclc_executor_add_timer(&executor_, &S.timer)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}

// ===== add subscriptions =====
int RosNodeManager::add_int32_subscription(const char* topic, Int32SubCb cb, void* user,
                                           rclc_executor_handle_invocation_t when){
  if(!inited_ || !cb) return -1;
  int idx=free_int32_sub(); if(idx<0) return -1;
  auto& S=int32_subs_[idx];
  if(rclc_subscription_init_default(&S.sub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32), topic)!=RCL_RET_OK) return -1;
  S.cb=cb; S.user=user; S.topic=topic; S.used=true;
  if(rclc_executor_add_subscription(&executor_, &S.sub, &S.msg, &RosNodeManager::int32_sub_adapter, when)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_float32_subscription(const char* topic, Float32SubCb cb, void* user,
                                             rclc_executor_handle_invocation_t when){
  if(!inited_ || !cb) return -1;
  int idx=free_float32_sub(); if(idx<0) return -1;
  auto& S=float32_subs_[idx];
  if(rclc_subscription_init_default(&S.sub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32), topic)!=RCL_RET_OK) return -1;
  S.cb=cb; S.user=user; S.topic=topic; S.used=true;
  if(rclc_executor_add_subscription(&executor_, &S.sub, &S.msg, &RosNodeManager::float32_sub_adapter, when)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_bool_subscription(const char* topic, BoolSubCb cb, void* user,
                                          rclc_executor_handle_invocation_t when){
  if(!inited_ || !cb) return -1;
  int idx=free_bool_sub(); if(idx<0) return -1;
  auto& S=bool_subs_[idx];
  if(rclc_subscription_init_default(&S.sub,&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool), topic)!=RCL_RET_OK) return -1;
  S.cb=cb; S.user=user; S.topic=topic; S.used=true;
  if(rclc_executor_add_subscription(&executor_, &S.sub, &S.msg, &RosNodeManager::bool_sub_adapter, when)!=RCL_RET_OK){ S.used=false; return -1; }
  return idx;
}
int RosNodeManager::add_string_subscription(const char* topic, StringSubCb cb, void* user,
                                            rclc_executor_handle_invocation_t when){
  if(!inited_ || !cb) return -1;
  int idx=free_string_sub(); if(idx<0) return -1;
  auto& S=string_subs_[idx];

  // ★ 用 BestEffort（避免 QoS 不配）
  if(rclc_subscription_init_best_effort(&S.sub,&node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,String), topic)!=RCL_RET_OK) return -1;

  // ✅ 在此時才初始化並預配置 buffer
  if(!std_msgs__msg__String__init(&S.msg)) return -1;
  S.msg_inited = true;
  rosidl_runtime_c__String__assignn(&S.msg.data, "", ROSN_STRING_PREALLOC);

  S.cb=cb; S.user=user; S.topic=topic; S.used=true;
  if(rclc_executor_add_subscription(&executor_, &S.sub, &S.msg, &RosNodeManager::string_sub_adapter, when)!=RCL_RET_OK){
    S.used=false; return -1;
  }
  return idx;
}

// ===== spin adapters =====
void RosNodeManager::timer_adapter(rcl_timer_t* t, int64_t){ if(s_instance) s_instance->on_timer(t); }
void RosNodeManager::int32_sub_adapter (const void* m){ if(s_instance) s_instance->on_int32_msg (m); }
void RosNodeManager::float32_sub_adapter(const void* m){ if(s_instance) s_instance->on_float32_msg(m); }
void RosNodeManager::bool_sub_adapter   (const void* m){ if(s_instance) s_instance->on_bool_msg   (m); }
void RosNodeManager::string_sub_adapter (const void* m){ if(s_instance) s_instance->on_string_msg (m); }

// ===== on_timer → 找到對應 pub slot 發佈 =====
void RosNodeManager::on_timer(rcl_timer_t* t){
  for(int i=0;i<ROSN_MAX_INT32_PUB;i++){ auto& S=int32_pubs_[i];
    if(S.used && &S.timer==t){ int32_t v=S.provider?S.provider():0; S.msg.data=v; (void)rcl_publish(&S.pub,&S.msg,NULL); return; } }
  for(int i=0;i<ROSN_MAX_FLOAT32_PUB;i++){ auto& S=float32_pubs_[i];
    if(S.used && &S.timer==t){ float v=S.provider?S.provider():0.0f; S.msg.data=v; (void)rcl_publish(&S.pub,&S.msg,NULL); return; } }
  for(int i=0;i<ROSN_MAX_BOOL_PUB;i++){ auto& S=bool_pubs_[i];
    if(S.used && &S.timer==t){ bool v=S.provider?S.provider():false; S.msg.data=v; (void)rcl_publish(&S.pub,&S.msg,NULL); return; } }
  for(int i=0;i<ROSN_MAX_STRING_PUB;i++){ auto& S=string_pubs_[i];
    if(S.used && &S.timer==t){
      const char* txt=S.provider?S.provider():"";
      if (S.msg_inited) { (void)rosidl_runtime_c__String__assign(&S.msg.data, txt); }
      (void)rcl_publish(&S.pub,&S.msg,NULL); return;
    } }
}

// ===== on_*_msg → 由 msg 指標找 slot 並呼叫使用者 cb =====
void RosNodeManager::on_int32_msg(const void* msgin){
  auto M=(const std_msgs__msg__Int32*)msgin;
  for(int i=0;i<ROSN_MAX_INT32_SUB;i++){ auto& S=int32_subs_[i]; if(S.used && &S.msg==M){ if(S.cb) S.cb(M,S.user); return; } }
}
void RosNodeManager::on_float32_msg(const void* msgin){
  auto M=(const std_msgs__msg__Float32*)msgin;
  for(int i=0;i<ROSN_MAX_FLOAT32_SUB;i++){ auto& S=float32_subs_[i]; if(S.used && &S.msg==M){ if(S.cb) S.cb(M,S.user); return; } }
}
void RosNodeManager::on_bool_msg(const void* msgin){
  auto M=(const std_msgs__msg__Bool*)msgin;
  for(int i=0;i<ROSN_MAX_BOOL_SUB;i++){ auto& S=bool_subs_[i]; if(S.used && &S.msg==M){ if(S.cb) S.cb(M,S.user); return; } }
}
void RosNodeManager::on_string_msg(const void* msgin){
  auto M=(const std_msgs__msg__String*)msgin;
  for(int i=0;i<ROSN_MAX_STRING_SUB;i++){ auto& S=string_subs_[i]; if(S.used && &S.msg==M){ if(S.cb) S.cb(M,S.user); return; } }
}
