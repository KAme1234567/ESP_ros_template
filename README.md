
# ESP32 micro-ROS 模組化開發模板

本專案提供一套模組化架構，讓你可以快速在 ESP32 上實現 micro-ROS 的發佈（Publisher）與訂閱（Subscriber）功能，支援 Wi-Fi fallback、多 topic、快速開發與團隊協作。

---

## 📦 專案目錄結構

```
lib/
├── RosWiFiHelper/       # Wi-Fi 自動連線與 micro-ROS 傳輸設定
├── RosPublisher/        # 發佈封裝（支援 timer 與 msg 型別）
├── RosSubscriber/       # 訂閱封裝（支援 callback 與 msg 型別）
include/
└── config.h             # topic 名、Wi-Fi、Agent 設定集中管理
src/
└── main.cpp             # 主程式（範例/可切換）
```

---

## ⚙️ 功能特點

- ✅ **Wi-Fi fallback**：主網路失敗會自動轉備用網路（如 AUV 的 AP 模式）
- ✅ **簡易初始化**：`publisher.init()` / `subscriber.init()` 一行搞定
- ✅ **模組獨立**：每個模組專責處理初始化與執行，清楚明瞭
- ✅ **支援多情境**：快速建立 pub/sub 範例與測試程式
- ✅ **團隊協作佳**：config 設定統一，程式結構清楚，便於多人開發

---

## 🚀 快速使用方式

### 1️⃣ 設定 Wi-Fi / topic 名稱（`include/config.h`）

```cpp
#define MAIN_SSID "your-wifi"
#define MAIN_PASS "your-password"
#define PUB_TOPIC "your_topic"
#define SUB_TOPIC "your_topic"
```

---

### 2️⃣ 發佈（Publisher）範例

```cpp
RosPublisher publisher;
publisher.init("node_name", PUB_TOPIC, timer_callback, 1000);

void timer_callback(rcl_timer_t*, int64_t) {
  publisher.msg.data = 123;
  rcl_publish(&publisher.publisher, &publisher.msg, NULL);
}
```

---

### 3️⃣ 訂閱（Subscriber）範例

```cpp
RosSubscriber subscriber;
subscriber.init("node_name", SUB_TOPIC, callback);

void callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  Serial.println(msg->data);
}
```

---

## 🧪 支援情境

- ✔️ 發佈開關狀態
- ✔️ 接收並控制 LED / 馬達
- ✔️ 雙向通訊（同時 pub + sub）
- ✔️ 支援多 ESP32 協同控制

---

## 🛠️ 需求與環境

- PlatformIO + VSCode
- ESP32（micro-ROS 支援開發板）
- micro_ros_platformio 套件
- ROS 2 Jazzy / Humble（搭配 micro-ROS Agent）

---

## ✨ 感謝

由 [國立高雄科技大學海事資訊科技系] 製作，適用於 AUV / IoT / ROS 開發專案。
