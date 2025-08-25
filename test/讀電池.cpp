#include <Arduino.h>

const int analogPin1 = 32; // 電池1輸入
const int analogPin2 = 33; // 電池2輸入

const float R1 = 10000.0;  // 上拉電阻
const float R2 = 2000.0;   // 下拉電阻
const float Vref = 3.3;    // ESP32 參考電壓

int estimateBatteryPercent(float voltage) {
  if (voltage >= 12.6) return 100;
  else if (voltage >= 12.4) return 90;
  else if (voltage >= 12.2) return 80;
  else if (voltage >= 12.0) return 70;
  else if (voltage >= 11.8) return 60;
  else if (voltage >= 11.6) return 50;
  else if (voltage >= 11.4) return 44;
  else if (voltage >= 11.2) return 35;
  else if (voltage >= 11.0) return 25;
  else return 0;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 預設為 12-bit
  Serial.println("🔋 開始監測兩顆 LiPo 電池");
}

void loop() {
  int raw1 = analogRead(analogPin1);
  int raw2 = analogRead(analogPin2);
  const float calibrationFactor1 = 1.134; // 你可以現場對照修
  const float calibrationFactor2 = 1.134;

 

  float voltageADC1 = raw1 * Vref / 4095.0;
  float voltageADC2 = raw2 * Vref / 4095.0;

  float voltageBat1 = voltageADC1 * (R1 + R2) / R2 * calibrationFactor1;
  float voltageBat2 = voltageADC2 * (R1 + R2) / R2 * calibrationFactor2;
  int percent1 = estimateBatteryPercent(voltageBat1);
  int percent2 = estimateBatteryPercent(voltageBat2);

  Serial.print("🔋 電池 1：");
  Serial.print(voltageBat1, 2);
  Serial.print(" V ｜ 剩餘：");
  Serial.print(percent1);
  Serial.println(" %");

  Serial.print("🔋 電池 2：");
  Serial.print(voltageBat2, 2);
  Serial.print(" V ｜ 剩餘：");
  Serial.print(percent2);
  Serial.println(" %");

  Serial.println("—————————————");

  delay(1000);
}
