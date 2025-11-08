// ESP32 Minimal DShot600 + Telemetry Sniffer (No ROS)
// 連線：ESC-CH1 -> GPIO26（信號），ESC-TX -> GPIO16（遙測），GND 共地，ESC 接電池
// BLHeliSuite32：該路設 DShot600、Telemetry=ON（允許反送TLM）
//
// 安全提示：測試時先拆槳或固定住，避免意外。

#include <Arduino.h>
#include "driver/rmt.h"

// ====== PIN 設定 ======
static const gpio_num_t MOTOR_PIN  = GPIO_NUM_26; // ESC 信號腳
static const int        TLM_RX_PIN = 16;          // 接 ESC 的 TX（UART RX，GPIO16 = RX2）

// ====== DShot 參數 (600kbit/s) ======
// APB 80MHz，RMT clk_div = 2 -> 40MHz -> 1 tick = 25ns
// 1 bit 週期約 1.667us -> 約 66~67 ticks
// '1': 高 75%（~1.25us=50 ticks），低 25%（~0.417us=16-17 ticks）
// '0': 高 37.5%（~0.625us=25 ticks），低 62.5%（~1.042us=41-42 ticks）
static const rmt_channel_t RMT_CH     = RMT_CHANNEL_0;
static const int           RMT_CLK_DIV = 2;   // 80MHz / 2 -> 40MHz (tick=25ns)
static const uint16_t      TICKS_ONE_HI  = 50;
static const uint16_t      TICKS_ONE_LO  = 17;
static const uint16_t      TICKS_ZERO_HI = 25;
static const uint16_t      TICKS_ZERO_LO = 42;

static const uint16_t DSHOT_MIN = 48;    // 0~47 特殊命令；48~2047 = 油門
static const uint16_t DSHOT_MAX = 2047;

// ====== UART (Telemetry) 設定 ======
HardwareSerial TLM_Serial(1);            // UART1（RX=GPIO16, TX=GPIO17）
static const uint32_t TLM_BAUD = 115200;

// ====== 工具：計算 4-bit 校驗 ======
static uint8_t dshot_checksum(uint16_t value11b, bool telemBit) {
  uint16_t v = (value11b << 1) | (telemBit ? 1 : 0); // 12 bits
  uint8_t csum = 0;
  for (int i = 0; i < 3; ++i) {
    csum ^= (v & 0xF);
    v >>= 4;
  }
  return (uint8_t)(csum & 0xF);
}

// ====== 建 frame → RMT 發送 ======
static void dshot_send(uint16_t throttle, bool telem) {
  if (throttle < DSHOT_MIN) throttle = DSHOT_MIN;
  if (throttle > DSHOT_MAX) throttle = DSHOT_MAX;

  const uint16_t payload = (throttle << 1) | (telem ? 1 : 0); // 12 bits
  const uint8_t  csum    = dshot_checksum(throttle, telem);   // 4 bits
  const uint16_t frame   = (payload << 4) | csum;             // 16 bits，MSB first

  rmt_item32_t items[16];
  for (int i = 15; i >= 0; --i) {
    const bool bit = (frame >> i) & 0x1;
    const int idx = 15 - i;
    if (bit) {
      items[idx].level0    = 1;
      items[idx].duration0 = TICKS_ONE_HI;
      items[idx].level1    = 0;
      items[idx].duration1 = TICKS_ONE_LO;
    } else {
      items[idx].level0    = 1;
      items[idx].duration0 = TICKS_ZERO_HI;
      items[idx].level1    = 0;
      items[idx].duration1 = TICKS_ZERO_LO;
    }
  }

  // 發送一個 frame（blocking=true 等 TX FIFO 清空）
  rmt_write_items(RMT_CH, items, 16, true);
  // 若需要可等 TX 完成：
  // rmt_wait_tx_done(RMT_CH, pdMS_TO_TICKS(5));
}

// ====== 將百分比轉 DShot 油門（48~2047） ======
static uint16_t pct_to_dshot(int pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return DSHOT_MIN + (uint16_t)((DSHOT_MAX - DSHOT_MIN) * (pct / 100.0f));
}

// ====== 讀 Telemetry：印 raw，嘗試解析 16 位小端片段 ======
static void read_tlm_once(uint32_t micros_idle_gap = 3000) {
  static uint8_t  buf[256];
  static uint32_t n = 0;
  static uint32_t last_us = 0;

  while (TLM_Serial.available()) {
    if (n < sizeof(buf)) buf[n++] = (uint8_t)TLM_Serial.read();
    last_us = micros();
  }

  if (n > 0 && (micros() - last_us) > micros_idle_gap) {
    Serial.printf("\n[TLM len=%u]\n", n);
    for (uint32_t i = 0; i < n; ++i) {
      if ((i % 16) == 0) Serial.print("[RAW ] ");
      if (buf[i] < 16) Serial.print('0');
      Serial.print(buf[i], HEX);
      Serial.print(' ');
      if ((i % 16) == 15) Serial.println();
    }

    if ((n % 16) == 0) {
      for (uint32_t off = 0; off < n; off += 16) {
        uint16_t w[8];
        for (int k = 0; k < 8; ++k) {
          w[k] = (uint16_t)buf[off + 2 * k] | ((uint16_t)buf[off + 2 * k + 1] << 8);
        }
        Serial.printf("[LE  ] w0=%u w1=%u w2=%u w3=%u w4=%u w5=%u w6=%u w7=%u\n",
                      w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
        // 觀察哪一欄隨油門合理變動，再決定 eRPM 來源（常見是某個 16-bit 欄位）
      }
    }
    n = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 DShot600 + Telemetry Minimal Test ==");

  // RMT for DShot
  rmt_config_t rmt_tx = {};
  rmt_tx.rmt_mode                 = RMT_MODE_TX;
  rmt_tx.channel                  = RMT_CH;
  rmt_tx.gpio_num                 = MOTOR_PIN;
  rmt_tx.mem_block_num            = 1;
  rmt_tx.tx_config.loop_en        = false;
  rmt_tx.tx_config.carrier_en     = false;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_tx.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;
  rmt_tx.clk_div                  = RMT_CLK_DIV; // tick = 25ns
  ESP_ERROR_CHECK(rmt_config(&rmt_tx));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_CH, 0, 0));

  // UART for Telemetry（先用非反相；若全 0 或雜訊再改為反相）
  pinMode(TLM_RX_PIN, INPUT); // 浮接避免外拉衝突；若需要可外接 10K 上拉
  TLM_Serial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, -1);
  // 若示波器看到信號為負邏輯（閒置低、脈波向上）再打開：
  // TLM_Serial.setRxInvert(true);

  // 給 ESC 一點上電穩定時間
  delay(500);
  Serial.println("Ready. Ensure ESC powered, Protocol=DShot600, Telemetry=ON.");
}

void loop() {
  // 簡單油門循環：40% → 50% → 60%，每段 1 秒
  static uint32_t phase_ms = 0;
  static int stage = 0;
  const uint32_t now = millis();

  int pct = (stage == 0) ? 40 : (stage == 1) ? 50 : 60;
  const uint16_t dval = pct_to_dshot(pct);

  // 每 ~2ms 發一次 DShot（帶 T 位元，要求 Telemetry）
  static uint32_t last_tx_us = 0;
  if ((micros() - last_tx_us) >= 2000) {
    dshot_send(dval, /*telemetry*/ true);
    last_tx_us = micros();
  }

  if ((now - phase_ms) >= 1000) {
    Serial.printf("[THROTTLE %%] %d%% (DSHOT=%u)\n", pct, dval);
    stage = (stage + 1) % 3;
    phase_ms = now;
  }

  // 抓 Telemetry
  read_tlm_once(3000);

  delay(1);
}
