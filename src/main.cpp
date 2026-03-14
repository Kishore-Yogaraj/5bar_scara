#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ready!");
}

void loop() {
  delay(1000);
  Serial.println("running...");
}