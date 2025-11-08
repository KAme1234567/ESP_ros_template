// ESP32 → CrossBL-32 4in1 ESC — DShot600 + Telemetry(8E2) with Motor Demux & RPM/VI/mAh
// -----------------------------------------------------------------------------
// Wiring
//   ESC Signal(Ch1) -> ESP32 GPIO18 (可改 26/25/21 …)
//   ESC Telemetry(TX) -> ESP32 GPIO16 (UART1 RX1)
// Power
//   ESC 必須接電池、GND 共地
// BLHeli_32
//   Protocol = DShot600, BiDirectional DShot = OFF, Telemetry = ON, Motor Stop = OFF
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include "driver/rmt.h"
#include "esp_err.h"
#include "HardwareSerial.h"

// ========= Pins =========
static const gpio_num_t MOTOR_PIN = GPIO_NUM_18;
static const int        TLM_RX_PIN = 16; // ESC TX -> ESP32 RX1

// ========= Telemetry UART =========
static const uint32_t TLM_BAUD = 115200;
#define TLM_SERIAL_MODE SERIAL_8E2
#define TLM_INVERT_RX   false
HardwareSerial TLM_Serial(1);

// ========= Motor / scaling =========
static const int  TARGET_MOTOR   = 0;  // 只取第幾顆馬達 (0~3)，你現在只接一顆，通常就是 0
static const int  POLE_PAIRS     = 7;  // 14 極 => 7
static const int  ERPM_SCALE     = 1;  // 若 eRPM 偏小，設 100；偏大則調 1/10…依實測
static const bool FORCE_MAP      = false; // true=使用手動欄位索引，false=自動辨識

// 若 FORCE_MAP=true，手動指定每個欄位在 w[0..7] 的索引；不確定就先用 -1
static const int IDX_VOLT = -1;  // mV
static const int IDX_TEMP = -1;  // °C
static const int IDX_CURR = -1;  // mA
static const int IDX_MAH  = -1;  // mAh
static const int IDX_ERPM = -1;  // eRPM（未除以 POLE_PAIRS、未乘 ERPM_SCALE）
static const int IDX_CRC  = -1;  // CRC / 保留

// ========= DShot600 Timing (APB 80MHz) =========
static const rmt_channel_t RMT_CH      = RMT_CHANNEL_0;
static const int           RMT_CLK_DIV = 2;   // 40MHz → 25ns/tick
static const uint16_t      TICKS_ONE_HI   = 50;
static const uint16_t      TICKS_ONE_LO   = 17;
static const uint16_t      TICKS_ZERO_HI  = 25;
static const uint16_t      TICKS_ZERO_LO  = 41;

static const uint16_t DSHOT_MIN = 48;    // 0..47 commands
static const uint16_t DSHOT_MAX = 2047;

// ========= Helpers (DShot) =========
static uint8_t dshot_checksum(uint16_t value11b, bool telemBit){
  uint16_t v = (value11b << 1) | (telemBit ? 1 : 0);
  uint8_t csum = 0;
  for (int i = 0; i < 3; ++i){ csum ^= (v & 0xF); v >>= 4; }
  return (uint8_t)(csum & 0xF);
}

static inline uint16_t pct_to_dshot(int pct){
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return DSHOT_MIN + (uint16_t)((DSHOT_MAX - DSHOT_MIN) * (pct / 100.0f));
}

static void dshot_send(uint16_t throttle, bool telem){
  if (throttle < DSHOT_MIN) throttle = DSHOT_MIN;
  if (throttle > DSHOT_MAX) throttle = DSHOT_MAX;

  const uint16_t payload = (throttle << 1) | (telem ? 1 : 0);
  const uint8_t  csum    = dshot_checksum(throttle, telem);
  const uint16_t frame   = (payload << 4) | csum;

  rmt_item32_t items[16];
  for (int i = 15; i >= 0; --i){
    const bool bit = (frame >> i) & 0x1;
    const int idx = 15 - i;
    items[idx].level0    = 1;
    items[idx].duration0 = bit ? TICKS_ONE_HI  : TICKS_ZERO_HI;
    items[idx].level1    = 0;
    items[idx].duration1 = bit ? TICKS_ONE_LO  : TICKS_ZERO_LO;
  }
  rmt_write_items(RMT_CH, items, 16, true);
}

static void dshot_command(uint8_t cmd){
  if (cmd > 47) cmd = 47;
  uint16_t value11b = cmd;
  uint8_t csum = dshot_checksum(value11b, false);
  uint16_t frame = ((value11b << 1) | 0) << 4 | csum;

  rmt_item32_t items[16];
  for (int i = 15; i >= 0; --i){
    bool bit = (frame >> i) & 1;
    int idx = 15 - i;
    items[idx].level0    = 1;
    items[idx].duration0 = bit ? TICKS_ONE_HI : TICKS_ZERO_HI;
    items[idx].level1    = 0;
    items[idx].duration1 = bit ? TICKS_ONE_LO : TICKS_ZERO_LO;
  }
  rmt_write_items(RMT_CH, items, 16, true);
}

// ========= Telemetry parsing =========
static uint32_t tlm_pkt_count = 0;
static volatile float g_rpm_ema = 0.f;
static const float    RPM_ALPHA = 0.30f;

struct Telemetry {
  int motor = -1;
  int16_t t_c = -1;       // °C
  int32_t v_mv = -1;      // mV
  int32_t i_ma = -1;      // mA
  int32_t mah  = -1;      // mAh
  int32_t erpm = -1;      // electrical RPM (raw)
  uint16_t crc = 0;       // raw crc/leftover
};

// 粗略範圍（幫助自動辨識）
static bool looks_voltage(uint16_t w){ return w >= 6000 && w <= 26000; }       // 6–26V
static bool looks_temp   (uint16_t w){ return w <= 200; }                       // 0–200°C
static bool looks_current(uint16_t w){ return w <= 20000; }                     // 0–20A (mA)
static bool looks_mah    (uint16_t w){ return w <= 65000; }                     // 0–65Ah 累計
static bool looks_erpm   (uint16_t w){ return w <= 120000; }                    // 0–120k eRPM
static bool looks_hdr78xx(uint16_t w){ return (w & 0xFF00) == 0x7800; }         // 0x78xx 標記
static int  get_motor_id (uint16_t w){ return (int)(w & 0x0003); }              // 低 2 bits

// 依值域自動找欄位（CrossBL-32：順序會旋轉，故只能看值域）
static void auto_map_and_fill(const uint16_t w[8], Telemetry& out){
  // 先找 header & motor
  int hdr_idx = -1;
  for (int k = 0; k < 8; ++k) if (looks_hdr78xx(w[k])) { hdr_idx = k; break; }
  if (hdr_idx >= 0) out.motor = get_motor_id(w[hdr_idx]);

  // 先找電壓（最好辨認）
  int vi = -1;
  for (int k = 0; k < 8; ++k) if (looks_voltage(w[k])) { vi = k; break; }
  if (vi >= 0) out.v_mv = (int32_t)w[vi];

  // 溫度
  int ti = -1;
  for (int k = 0; k < 8; ++k) if (k!=vi && looks_temp(w[k])) { ti = k; break; }
  if (ti >= 0) out.t_c = (int16_t)w[ti];

  // 電流
  int ii = -1;
  for (int k = 0; k < 8; ++k) if (k!=vi && k!=ti && looks_current(w[k])) { ii = k; break; }
  if (ii >= 0) out.i_ma = (int32_t)w[ii];

  // eRPM（用剩下的裡面挑最像的；避開 0x78xx、電壓/溫度/電流）
  int ei = -1;
  for (int k = 0; k < 8; ++k){
    if (k==vi || k==ti || k==ii) continue;
    if (looks_hdr78xx(w[k])) continue;
    // 偏挑大一點且變化幅度比較活躍的
    if (looks_erpm(w[k])) { ei = k; }
  }
  if (ei >= 0) out.erpm = (int32_t)w[ei];

  // mAh（剩下合理者）
  int mi = -1;
  for (int k = 0; k < 8; ++k){
    if (k==vi || k==ti || k==ii || k==ei) continue;
    if (!looks_hdr78xx(w[k]) && looks_mah(w[k])) { mi = k; break; }
  }
  if (mi >= 0) out.mah = (int32_t)w[mi];

  // 其餘當 CRC/保留
  for (int k = 0; k < 8; ++k){
    if (k==vi || k==ti || k==ii || k==ei || k==mi) continue;
    out.crc = w[k];
    break;
  }
}

// 手動索引寫入
static void manual_fill(const uint16_t w[8], Telemetry& out){
  // motor from header（若你知道 header 索引也可手動抓）
  for (int k = 0; k < 8; ++k) if (looks_hdr78xx(w[k])) { out.motor = get_motor_id(w[k]); break; }
  if (IDX_VOLT >= 0) out.v_mv = w[IDX_VOLT];
  if (IDX_TEMP >= 0) out.t_c  = (int16_t)w[IDX_TEMP];
  if (IDX_CURR >= 0) out.i_ma = w[IDX_CURR];
  if (IDX_MAH  >= 0) out.mah  = w[IDX_MAH];
  if (IDX_ERPM >= 0) out.erpm = w[IDX_ERPM];
  if (IDX_CRC  >= 0) out.crc  = w[IDX_CRC];
}

// 每包 16 bytes
static void on_tlm_frame(uint8_t* buf, uint32_t nBytes){
  if (nBytes % 16 != 0) return;

  for (uint32_t off = 0; off + 16 <= nBytes; off += 16){
    uint16_t w[8];
    for (int k = 0; k < 8; ++k){
      w[k] = (uint16_t)buf[off + 2*k] | ((uint16_t)buf[off + 2*k + 1] << 8);
    }

    Telemetry t;
    if (FORCE_MAP) manual_fill(w, t);
    else           auto_map_and_fill(w, t);

    // 只處理目標馬達（沒有 header 時，motor 仍可=-1，這種包就跳過）
    if (t.motor != -1 && t.motor != TARGET_MOTOR) continue;

    // eRPM → RPM（含 scale）
    float rpm = 0.f;
    if (t.erpm >= 0){
      const float erpm_scaled = (float)t.erpm * (float)ERPM_SCALE;
      if (POLE_PAIRS > 0) rpm = erpm_scaled / (float)POLE_PAIRS;
    }
    if (g_rpm_ema <= 1e-6f) g_rpm_ema = rpm;
    else g_rpm_ema = RPM_ALPHA * rpm + (1.f - RPM_ALPHA) * g_rpm_ema;

    tlm_pkt_count++;

    // 調試列印（觀察欄位與 motor）
    Serial.printf("[TLM] M%d  w0=%u w1=%u w2=%u w3=%u w4=%u w5=%u w6=%u w7=%u  | V=%ldmV I=%ldmA mAh=%ld T=%dC eRPM=%ld CRC=0x%04X  RPM≈%.1f\n",
      t.motor, w[0],w[1],w[2],w[3],w[4],w[5],w[6],w[7],
      (long)t.v_mv,(long)t.i_ma,(long)t.mah,(int)t.t_c,(long)t.erpm,(unsigned)t.crc, rpm);
  }
}

// 以 idle gap 分包
static void tlm_poll(uint32_t micros_idle_gap = 3000){
  static uint8_t  buf[256];
  static uint32_t n = 0;
  static uint32_t last_us = 0;

  while (TLM_Serial.available()){
    if (n < sizeof(buf)) buf[n++] = (uint8_t)TLM_Serial.read();
    last_us = micros();
  }
  if (n > 0 && (micros() - last_us) > micros_idle_gap){
    on_tlm_frame(buf, n);
    n = 0;
  }
}

// ========= Setup / Loop =========
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 DShot600 + BLHeli32 Telemetry (4in1 demux) ==");

  // RMT TX
  rmt_config_t rmt_tx = {};
  rmt_tx.rmt_mode                 = RMT_MODE_TX;
  rmt_tx.channel                  = RMT_CH;
  rmt_tx.gpio_num                 = MOTOR_PIN;
  rmt_tx.mem_block_num            = 1;
  rmt_tx.tx_config.loop_en        = false;
  rmt_tx.tx_config.carrier_en     = false;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_tx.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;
  rmt_tx.clk_div                  = RMT_CLK_DIV;
  ESP_ERROR_CHECK(rmt_config(&rmt_tx));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_CH, 0, 0));

  // UART1 for TLM
  pinMode(TLM_RX_PIN, INPUT);
  TLM_Serial.begin(TLM_BAUD, TLM_SERIAL_MODE, TLM_RX_PIN, -1);
  TLM_Serial.setRxInvert(TLM_INVERT_RX);

  Serial.printf("TARGET_MOTOR=%d, POLE_PAIRS=%d, ERPM_SCALE=%d, FORCE_MAP=%s\n",
                TARGET_MOTOR, POLE_PAIRS, ERPM_SCALE, FORCE_MAP ? "true":"false");
}

// 固定 25% 油門，1kHz 送；每秒摘要
void loop(){
  static uint32_t t0 = millis();
  const uint32_t t = millis() - t0;

  static uint32_t last_tx_us = 0;
  uint32_t now_us = micros();
  if ((now_us - last_tx_us) >= 1000){
    dshot_send(pct_to_dshot(25), true); // 問 TLM
    last_tx_us = now_us;
  }

  // 啟動初期嗶聲確認
  if (t > 2000 && t < 2600){
    static uint32_t last_beep_ms = 0;
    if (millis() - last_beep_ms > 80){
      dshot_command(0);
      last_beep_ms = millis();
    }
  }

  tlm_poll(3000);

  // 每秒摘要
  static uint32_t last_log = 0;
  if (millis() - last_log > 1000){
    // 這裡只列目標馬達的 EMA RPM；電壓/電流/溫度/耗電會在 [TLM] 行裡持續印
    Serial.printf("[STATUS] throttle=25%%, pkts=%lu, RPM(EMA)≈%.1f (motor=%d)\n",
                  (unsigned long)tlm_pkt_count, g_rpm_ema, TARGET_MOTOR);
    last_log = millis();
  }

  delay(1);
}
