// RosWiFiHelper.cpp
#include "RosWiFiHelper.h"

#include "config.h"
  // ✅ 加這行，且用相對路徑引入 config.h

bool connectToWiFi(const char* ssid, const char* password, IPAddress local_ip, IPAddress gateway, IPAddress subnet, int max_attempts) {
  WiFi.mode(WIFI_STA);
  WiFi.config(local_ip, gateway, subnet);
  WiFi.begin(ssid, password);

  for (int i = 0; i < max_attempts; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("✅ Connected: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(1000);
  }
  Serial.println("❌ Wi-Fi Failed");
  return false;
}

void setup_wifi_fallback() {
  if (connectToWiFi(MAIN_SSID, MAIN_PASS, MAIN_IP, MAIN_GATEWAY, MAIN_SUBNET)) {
    set_microros_wifi_transports(MAIN_SSID, MAIN_PASS, MAIN_AGENT, AGENT_PORT);
  } else if (connectToWiFi(BACKUP_SSID, BACKUP_PASS, BACKUP_IP, BACKUP_GATEWAY, BACKUP_SUBNET)) {
    set_microros_wifi_transports(BACKUP_SSID, BACKUP_PASS, BACKUP_AGENT, AGENT_PORT);
  } else {
    Serial.println("❌ 無法連接任何 Wi-Fi，停機等待...");
    while (1) delay(1000);
  }
}