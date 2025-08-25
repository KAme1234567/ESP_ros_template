#include "RosPublisher.h"
#include <string.h>

<<<<<<< HEAD
RosPublisherManager* RosPublisherManager::s_instance = nullptr;
=======
template<typename MsgType>

bool RosPublisher<MsgType>::init(const char* node_name, const rosidl_message_type_support_t* ts, const char* topic_name, rcl_timer_callback_t cb, int interval_ms) {
  allocator = rcl_get_default_allocator();
>>>>>>> 30acb290d12f5dac78a29d347b8c70b6f273b1af

RosPublisherManager::RosPublisherManager()
: allocator_(rcl_get_default_allocator()),
  inited_(false)
{
  memset(&support_,  0, sizeof(support_));
  memset(&node_,     0, sizeof(node_));
  memset(&executor_, 0, sizeof(executor_));

<<<<<<< HEAD
  for (int i = 0; i < ROSP_MAX_INT32; ++i) {
    int32_slots_[i].used = false;
    memset(&int32_slots_[i].pub,   0, sizeof(rcl_publisher_t));
    memset(&int32_slots_[i].timer, 0, sizeof(rcl_timer_t));
    int32_slots_[i].msg.data = 0;
    int32_slots_[i].provider = nullptr;
    int32_slots_[i].topic = nullptr;
  }
  for (int i = 0; i < ROSP_MAX_FLOAT32; ++i) {
    float32_slots_[i].used = false;
    memset(&float32_slots_[i].pub,   0, sizeof(rcl_publisher_t));
    memset(&float32_slots_[i].timer, 0, sizeof(rcl_timer_t));
    float32_slots_[i].msg.data = 0.0f;
    float32_slots_[i].provider = nullptr;
    float32_slots_[i].topic = nullptr;
  }
  for (int i = 0; i < ROSP_MAX_BOOL; ++i) {
    bool_slots_[i].used = false;
    memset(&bool_slots_[i].pub,   0, sizeof(rcl_publisher_t));
    memset(&bool_slots_[i].timer, 0, sizeof(rcl_timer_t));
    bool_slots_[i].msg.data = false;
    bool_slots_[i].provider = nullptr;
    bool_slots_[i].topic = nullptr;
  }
  for (int i = 0; i < ROSP_MAX_STRING; ++i) {
    string_slots_[i].used = false;
    memset(&string_slots_[i].pub,   0, sizeof(rcl_publisher_t));
    memset(&string_slots_[i].timer, 0, sizeof(rcl_timer_t));
    std_msgs__msg__String__init(&string_slots_[i].msg); // 初始化
    string_slots_[i].provider = nullptr;
    string_slots_[i].topic = nullptr;
  }
}

RosPublisherManager::~RosPublisherManager() {
  fini();
}

bool RosPublisherManager::init(const char* node_name) {
  if (inited_) return true;
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, node_name, "", &support_) != RCL_RET_OK) return false;

  // 預留容量：最大可能的 timer 數量（四種型別加總上限）
  const size_t max_timers = (ROSP_MAX_INT32 + ROSP_MAX_FLOAT32 + ROSP_MAX_BOOL + ROSP_MAX_STRING);
  if (rclc_executor_init(&executor_, &support_.context, max_timers, &allocator_) != RCL_RET_OK) return false;

  s_instance = this;
  inited_ = true;
=======
  // 初始化 msg.data（針對 String 類型）
  rosidl_runtime_c__String__init(&msg.data);
>>>>>>> 30acb290d12f5dac78a29d347b8c70b6f273b1af
  return true;
}

void RosPublisherManager::fini() {
  if (!inited_) return;

  // 清理各型別 slot
  for (int i = 0; i < ROSP_MAX_INT32; ++i) {
    if (int32_slots_[i].used) {
      rcl_timer_fini(&int32_slots_[i].timer);
      rcl_publisher_fini(&int32_slots_[i].pub, &node_);
      int32_slots_[i].used = false;
    }
  }
  for (int i = 0; i < ROSP_MAX_FLOAT32; ++i) {
    if (float32_slots_[i].used) {
      rcl_timer_fini(&float32_slots_[i].timer);
      rcl_publisher_fini(&float32_slots_[i].pub, &node_);
      float32_slots_[i].used = false;
    }
  }
  for (int i = 0; i < ROSP_MAX_BOOL; ++i) {
    if (bool_slots_[i].used) {
      rcl_timer_fini(&bool_slots_[i].timer);
      rcl_publisher_fini(&bool_slots_[i].pub, &node_);
      bool_slots_[i].used = false;
    }
  }
  for (int i = 0; i < ROSP_MAX_STRING; ++i) {
    if (string_slots_[i].used) {
      rcl_timer_fini(&string_slots_[i].timer);
      rcl_publisher_fini(&string_slots_[i].pub, &node_);
      string_slots_[i].used = false;
    }
    // 不論 used 與否，string msg 都做 fini（因為在建構時 init 了）
    std_msgs__msg__String__fini(&string_slots_[i].msg);
  }

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
  inited_ = false;
  s_instance = nullptr;
}

<<<<<<< HEAD
int RosPublisherManager::find_free_int32 () { for (int i=0;i<ROSP_MAX_INT32 ;++i) if (!int32_slots_[i].used)  return i; return -1; }
int RosPublisherManager::find_free_float32() { for (int i=0;i<ROSP_MAX_FLOAT32;++i) if (!float32_slots_[i].used) return i; return -1; }
int RosPublisherManager::find_free_bool  () { for (int i=0;i<ROSP_MAX_BOOL  ;++i) if (!bool_slots_[i].used)   return i; return -1; }
int RosPublisherManager::find_free_string() { for (int i=0;i<ROSP_MAX_STRING;++i) if (!string_slots_[i].used) return i; return -1; }

int RosPublisherManager::add_int32_publisher(const char* topic_name, uint32_t period_ms, Int32ProviderFn provider) {
  if (!inited_ || provider == nullptr) return -1;
  int idx = find_free_int32(); if (idx < 0) return -1;
  auto& S = int32_slots_[idx];

  if (rclc_publisher_init_default(&S.pub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), topic_name) != RCL_RET_OK) return -1;

  if (rclc_timer_init_default(&S.timer, &support_, RCL_MS_TO_NS(period_ms),
      &RosPublisherManager::timer_callback_adapter) != RCL_RET_OK) return -1;

  S.msg.data = 0;
  S.provider = provider;
  S.topic = topic_name;
  S.used = true;

  if (rclc_executor_add_timer(&executor_, &S.timer) != RCL_RET_OK) { S.used=false; return -1; }
  return idx;
}

int RosPublisherManager::add_float32_publisher(const char* topic_name, uint32_t period_ms, Float32ProviderFn provider) {
  if (!inited_ || provider == nullptr) return -1;
  int idx = find_free_float32(); if (idx < 0) return -1;
  auto& S = float32_slots_[idx];

  if (rclc_publisher_init_default(&S.pub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), topic_name) != RCL_RET_OK) return -1;

  if (rclc_timer_init_default(&S.timer, &support_, RCL_MS_TO_NS(period_ms),
      &RosPublisherManager::timer_callback_adapter) != RCL_RET_OK) return -1;

  S.msg.data = 0.0f;
  S.provider = provider;
  S.topic = topic_name;
  S.used = true;

  if (rclc_executor_add_timer(&executor_, &S.timer) != RCL_RET_OK) { S.used=false; return -1; }
  return idx;
}

int RosPublisherManager::add_bool_publisher(const char* topic_name, uint32_t period_ms, BoolProviderFn provider) {
  if (!inited_ || provider == nullptr) return -1;
  int idx = find_free_bool(); if (idx < 0) return -1;
  auto& S = bool_slots_[idx];

  if (rclc_publisher_init_default(&S.pub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), topic_name) != RCL_RET_OK) return -1;

  if (rclc_timer_init_default(&S.timer, &support_, RCL_MS_TO_NS(period_ms),
      &RosPublisherManager::timer_callback_adapter) != RCL_RET_OK) return -1;

  S.msg.data = false;
  S.provider = provider;
  S.topic = topic_name;
  S.used = true;

  if (rclc_executor_add_timer(&executor_, &S.timer) != RCL_RET_OK) { S.used=false; return -1; }
  return idx;
}

int RosPublisherManager::add_string_publisher(const char* topic_name, uint32_t period_ms, StringProviderFn provider) {
  if (!inited_ || provider == nullptr) return -1;
  int idx = find_free_string(); if (idx < 0) return -1;
  auto& S = string_slots_[idx];

  if (rclc_publisher_init_default(&S.pub, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name) != RCL_RET_OK) return -1;

  if (rclc_timer_init_default(&S.timer, &support_, RCL_MS_TO_NS(period_ms),
      &RosPublisherManager::timer_callback_adapter) != RCL_RET_OK) return -1;

  // 已在 ctor 先做過 std_msgs__msg__String__init(&S.msg)
  S.provider = provider;
  S.topic = topic_name;
  S.used = true;

  if (rclc_executor_add_timer(&executor_, &S.timer) != RCL_RET_OK) { S.used=false; return -1; }
  return idx;
}

void RosPublisherManager::spin_some(uint32_t timeout_ms) {
  if (!inited_) return;
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(timeout_ms));
}

// static
void RosPublisherManager::timer_callback_adapter(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (s_instance) s_instance->on_timer(timer);
}

void RosPublisherManager::on_timer(rcl_timer_t* timer) {
  // 依序檢查觸發來源在哪種 slot

  // Int32
  for (int i = 0; i < ROSP_MAX_INT32; ++i) {
    auto& S = int32_slots_[i];
    if (S.used && (&S.timer == timer)) {
      int32_t v = S.provider ? S.provider() : 0;
      S.msg.data = v;
      (void)rcl_publish(&S.pub, &S.msg, NULL);
      return;
    }
  }
  // Float32
  for (int i = 0; i < ROSP_MAX_FLOAT32; ++i) {
    auto& S = float32_slots_[i];
    if (S.used && (&S.timer == timer)) {
      float v = S.provider ? S.provider() : 0.0f;
      S.msg.data = v;
      (void)rcl_publish(&S.pub, &S.msg, NULL);
      return;
    }
  }
  // Bool
  for (int i = 0; i < ROSP_MAX_BOOL; ++i) {
    auto& S = bool_slots_[i];
    if (S.used && (&S.timer == timer)) {
      bool v = S.provider ? S.provider() : false;
      S.msg.data = v;
      (void)rcl_publish(&S.pub, &S.msg, NULL);
      return;
    }
  }
  // String
  for (int i = 0; i < ROSP_MAX_STRING; ++i) {
    auto& S = string_slots_[i];
    if (S.used && (&S.timer == timer)) {
      const char* txt = S.provider ? S.provider() : "";
      // 用官方 assign（會自動配置/覆寫）
      (void)rosidl_runtime_c__String__assign(&S.msg.data, txt);
      (void)rcl_publish(&S.pub, &S.msg, NULL);
      return;
    }
  }
}
=======
// 傳送 const char*（字串形式）
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, const char* value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":\"%s\"}", key, value);

  if (strlen(json_buffer) >= sizeof(json_buffer) - 1) {
    Serial.println("❌ JSON 太長，未送出");
    return;
  }

  if (!rosidl_runtime_c__String__assign(&msg.data, json_buffer)) {
    Serial.println("❌ String assign 失敗");
    return;
  }

  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.print("❌ Publish 失敗: ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
  } else {
    Serial.println("[SEND OK]");
  }
}

// 傳送 int
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, int value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":%d}", key, value);
  send(key, json_buffer);  // 重用字串版
}

// 傳送 float
template<typename MsgType>
void RosPublisher<MsgType>::send(const char* key, float value) {
  snprintf(json_buffer, sizeof(json_buffer), "{\"%s\":%.2f}", key, value);
  send(key, json_buffer);  // 重用字串版
}

// 強制實例化模板
template class RosPublisher<std_msgs__msg__String>;
>>>>>>> 30acb290d12f5dac78a29d347b8c70b6f273b1af
