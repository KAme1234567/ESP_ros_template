#include <Arduino.h>
#include "config.h"
#include <RosNodeManager.h>
#include "RosWiFiHelper.h"

// ===== Providers =====
int32_t read_battery_mv(){ return 11700; }
float   read_depth_m(){ static float d=0; d += 0.01f; return d; }
bool    read_armed(){ return true; }
const char* read_status(){ return "OK"; }

// ===== Sub callbacks =====
void on_mode(const std_msgs__msg__Int32* m, void*) {
  Serial.printf("[SUB] /cmd/mode = %d\n", (int)m->data);
}
void on_target_depth(const std_msgs__msg__Float32* m, void*) {
  Serial.printf("[SUB] /cmd/target_depth = %.2f\n", m->data);
}
void on_armed(const std_msgs__msg__Bool* m, void*) {
  Serial.printf("[SUB] /cmd/armed = %s\n", m->data ? "true" : "false");
}
void on_note(const std_msgs__msg__String* m, void*) {
  Serial.printf("[SUB] /cmd/note = %s\n", m->data.data ? m->data.data : "");
}

RosNodeManager node;

void setup(){
  Serial.begin(115200);
  delay(200);
  setup_wifi_fallback();

  if(!node.init(NODE_NAME)){
    Serial.println("RosNodeManager init failed!");
    return;
  }

  // === Publishers ===
  node.add_int32_publisher ("/battery/mv", 1000, &read_battery_mv);
  node.add_float32_publisher("/depth/m",    200, &read_depth_m);
  node.add_bool_publisher   ("/system/armed", 1000, &read_armed);
  node.add_string_publisher ("/system/status",1000, &read_status);

  // === Subscriptions ===
  node.add_int32_subscription ("/cmd/mode",          &on_mode);
  node.add_float32_subscription("/cmd/target_depth", &on_target_depth);
  node.add_bool_subscription   ("/cmd/armed",        &on_armed);
  node.add_string_subscription ("/cmd/note",         &on_note); // BestEffort + 預配置

  Serial.println("RosNodeManager ready.");
}

void loop(){
  node.spin_some(20);
}
