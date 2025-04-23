// RosWiFiHelper.h
#pragma once

#include <WiFi.h>
#include <micro_ros_platformio.h>
#include "config.h"

bool connectToWiFi(const char* ssid, const char* password, IPAddress local_ip, IPAddress gateway, IPAddress subnet, int max_attempts = 10);
void setup_wifi_fallback();