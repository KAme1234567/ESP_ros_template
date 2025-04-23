#include "config.h"
#include "RosWiFiHelper.h"
#include "RosPublisher.h"

RosPublisher publisher;

const int buttonPin1 = 2;
const int buttonPin2 = 4;
int last_state = -1;

void timer_callback(rcl_timer_t*, int64_t) {
  int b1 = digitalRead(buttonPin1);
  int b2 = digitalRead(buttonPin2);
  int state = (!b1 << 1) | (!b2);

  if (state != last_state) {
    publisher.msg.data = state;
    rcl_publish(&publisher.publisher, &publisher.msg, NULL);
    last_state = state;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  setup_wifi_fallback();
  publisher.init("button_pub_node", PUB_TOPIC, timer_callback, 1000);
}

void loop() {
  publisher.spin();
  delay(100);
}
