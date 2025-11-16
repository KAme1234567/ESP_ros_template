// config.h
#pragma once

#include <WiFi.h>
#include <IPAddress.h>

#ifndef AUV_POWER_MINI
#define AUV_POWER_MINI 0
#endif

#if AUV_POWER_MINI
#define NODE_NAME "esp32_system"
#endif
#if !AUV_POWER_MINI
#define NODE_NAME "esp32_power"
#endif


// ===== Wi-Fi 設定 =====
#define MAIN_SSID "auv-12345678"
#define MAIN_PASS "12345678"

#define MAIN_IP IPAddress(192, 168, 4, 27)
#define MAIN_GATEWAY IPAddress(192, 168, 4, 1)


#if AUV_POWER_MINI
#define MAIN_IP IPAddress(192, 168, 4, 30)
#endif
#if !AUV_POWER_MINI
#define MAIN_IP IPAddress(192, 168, 4, 31)
#endif


#define MAIN_SUBNET IPAddress(255, 255, 255, 0)
#define MAIN_AGENT IPAddress(192, 168, 4, 1)


#define BACKUP_SSID "備用無線網路"
#define BACKUP_PASS "123456789"
#define BACKUP_IP IPAddress(192, 168, 1, 151)
#define BACKUP_GATEWAY IPAddress(192, 168, 1, 1)

#define BACKUP_SUBNET IPAddress(255, 255, 255, 0)
#define BACKUP_AGENT IPAddress(192, 168, 1, 107)


#define AGENT_PORT 8888

