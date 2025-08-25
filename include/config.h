// config.h
#pragma once

#include <WiFi.h>
#include <IPAddress.h>
#define NODE_NAME "esp32_node_allinone"
// ===== Wi-Fi 設定 =====
// ⬅️ 原本是 MAIN 的，現在變成 BACKUP
#define MAIN_SSID "auv-12345678"
#define MAIN_PASS "12345678"
#define MAIN_IP IPAddress(192, 168, 4, 21)
#define MAIN_GATEWAY IPAddress(192, 168, 4, 1)
#define MAIN_SUBNET IPAddress(255, 255, 255, 0)
#define MAIN_AGENT IPAddress(192, 168, 4, 1)

// ⬅️ 原本是 BACKUP 的，現在變成 MAIN
#define BACKUP_SSID "Kame的無線網路"
#define BACKUP_PASS "Kame114514"
#define BACKUP_IP IPAddress(192, 168, 1, 151)
#define BACKUP_GATEWAY IPAddress(192, 168, 1, 1)
#define BACKUP_SUBNET IPAddress(255, 255, 255, 0)
#define BACKUP_AGENT IPAddress(192, 168, 1, 107)


#define AGENT_PORT 8888

// topic 名稱
#define PUB_TOPIC "/esp_TEST"
#define SUB_TOPIC "/motor/speed_percent"

