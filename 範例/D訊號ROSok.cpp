// ESP32 → BLHeli_32 ESC — DShot600（RMT） + ROS 控制（WiFi + RosNodeManager）
// -----------------------------------------------------------------------------
#include <Arduino.h>
extern "C" {
  #include "driver/rmt.h"
}
#include "esp_err.h"

#include <Wire.h>
#include "config.h"
#include <RosNodeManager.h>
#include "RosWiFiHelper.h"

// ================== 使用者可調參數 ==================
#define ESC_SIG_PIN          26             // DShot 輸出腳
#define RMT_CH               RMT_CHANNEL_0
#define SEND_HZ              1000           // DShot 發送頻率（建議 1kHz）
#define ARM_ZERO_MS          3000           // 上電後持續 0 油門時間（ms）
#define RAMP_STEP            1            // 每個 frame 油門最大變化量（0~2000）
#define DSHOT11_MAX         1000
//最大油門

// === ROS Topic 名稱 ===
static const char* TOPIC_THROTTLE = "/cmd/throttle_0_2000"; // std_msgs/Int32
static const char* TOPIC_ESTOP    = "/cmd/emergency_stop";  // std_msgs/Bool
static const char* TOPIC_IS_ARMED = "/esc/is_armed";        // std_msgs/Bool（回報）

// ================== DShot 時序（clk_div=8 → 10MHz, 0.1us/tick） ==================
// '1': 高 12, 低 5  → 1.7us
// '0': 高  6, 低 11 → 1.7us
static inline void dshot_make_item(rmt_item32_t* item, bool bit1) {
  if (bit1) { // '1'
    item->duration0 = 12; item->level0 = 1;
    item->duration1 = 5;  item->level1 = 0;
  } else {    // '0'
    item->duration0 = 6;  item->level0 = 1;
    item->duration1 = 11; item->level1 = 0;
  }
}

// 11-bit throttle + TLM + 4-bit XOR checksum
static uint16_t dshot_pack(uint16_t throttle11, bool req_tlm) {
  uint16_t packet = (throttle11 & 0x07FF) << 5;
  packet |= (req_tlm ? 1 : 0) << 4;
  uint16_t csum = 0, d = packet >> 4;
  for (int i = 0; i < 3; ++i) {
    csum ^= (d & 0xF);
    d >>= 4;
  }
  packet |= (csum & 0xF);
  return packet;
}

static void dshot_send_raw(uint16_t value11, bool req_tlm) {
  uint16_t pkt = dshot_pack(value11, req_tlm);
  rmt_item32_t items[16];
  for (int i = 0; i < 16; ++i) {
    bool bit1 = (pkt & (1 << (15 - i))) != 0;
    dshot_make_item(&items[i], bit1);
  }
  ESP_ERROR_CHECK(rmt_write_items(RMT_CH, items, 16, true));          // 阻塞送完
  ESP_ERROR_CHECK(rmt_wait_tx_done(RMT_CH, portMAX_DELAY));
}

// 把 0~2000 線性映射到 48~2047（<48 是命令保留）
static inline uint16_t map_0_2000_to_dshot11(int v) {
  if (v < 0) v = 0;
  if (v > 2000) v = 2000;
  int t = map(v, 0, 2000, 48, 2047);
  if (t < 48) t = 48;
  return (uint16_t)t;
}

static void setup_rmt() {
  rmt_config_t cfg = {};
  cfg.rmt_mode                 = RMT_MODE_TX;
  cfg.channel                  = RMT_CH;
  cfg.gpio_num                 = (gpio_num_t)ESC_SIG_PIN;
  cfg.mem_block_num            = 1;
  cfg.clk_div                  = 8; // 80MHz/8 = 10MHz (0.1us/tick)
  cfg.tx_config.loop_en        = false;
  cfg.tx_config.carrier_en     = false;
  cfg.tx_config.idle_output_en = true;
  cfg.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW; // DShot idle 低
  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_CH, 0, 0));
}

// ================== ROS Node Manager ==================
RosNodeManager node;

// 回報 is_armed publisher（使用簡單 getter 綁定）
static bool g_is_armed = false;
static bool read_is_armed() { return g_is_armed; }

// ================== 控制狀態 ==================
static volatile int  g_cmd_throttle = 0;      // 目標油門（0~2000）
static int           g_cur_throttle = 0;      // 當前輸出油門（做 ramp）
static volatile bool g_estop = false;         // 急停旗標（true → 立刻輸出 0）

// 訂閱回呼：油門（0~2000）
static void on_throttle_cmd(const std_msgs__msg__Int32* msg, void*) {
  int v = msg->data;
  if (v < 0) v = 0;
  if (v > DSHOT11_MAX ) v = DSHOT11_MAX ;
  // if (v > 2000) v = 2000;
  g_cmd_throttle = v;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
  Serial.printf("[ROS] throttle cmd = %d\n", g_cmd_throttle);
#endif
}

// 訂閱回呼：急停
static void on_estop_cmd(const std_msgs__msg__Bool* msg, void*) {
  g_estop = msg->data;
  if (g_estop) {
    g_cmd_throttle = 0;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG
    Serial.println("[ROS] EMERGENCY STOP");
#endif
  }
}

// ================== 主程式 ==================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 DShot600 + ROS throttle control ==");

  setup_rmt();

  // WiFi / ROS 連線
  setup_wifi_fallback();
  if (!node.init(NODE_NAME)) {
    Serial.println("ROS 初始化失敗");
    // 不 return，仍可用固定 0 油門保護
  }

  // ROS Publishers
  node.add_bool_publisher(TOPIC_IS_ARMED,  1000, &read_is_armed);

  // ROS Subscriptions
  node.add_int32_subscription(TOPIC_THROTTLE, &on_throttle_cmd);
  node.add_bool_subscription (TOPIC_ESTOP,    &on_estop_cmd);

  // 上電 Arm：固定 0 油門 ARM_ZERO_MS
  uint32_t t0 = millis();
  while (millis() - t0 < ARM_ZERO_MS) {
    dshot_send_raw(map_0_2000_to_dshot11(0), false);
    // 1kHz 間隔
    delayMicroseconds(1000000UL / SEND_HZ);
    // 非阻塞處理 ROS
    node.spin_some(0);
  }
  g_is_armed = true;
  Serial.println("Armed with 0 throttle. Ready for ROS commands.");
}

// 小工具：一步 ramp 往 g_cmd_throttle
static inline void ramp_step() {
  int target = g_cmd_throttle;
  if (g_estop) target = 0;

  if (g_cur_throttle < target) {
    g_cur_throttle += RAMP_STEP;
    if (g_cur_throttle > target) g_cur_throttle = target;
  } else if (g_cur_throttle > target) {
    g_cur_throttle -= RAMP_STEP;
    if (g_cur_throttle < target) g_cur_throttle = target;
  }
}

void loop() {
  // 每個 frame 做一次 ramp
  ramp_step();

  // 送出 DShot frame（不請求 TLM）
  uint16_t v11 = map_0_2000_to_dshot11(g_cur_throttle);
  dshot_send_raw(v11, false);

  // 1kHz 間隔
  delayMicroseconds(1000000UL / SEND_HZ);

  // 快速處理 ROS 事件（非阻塞）
  node.spin_some(0);
}
