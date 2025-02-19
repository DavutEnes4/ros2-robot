#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("PlatformIO ile ESP32C3 çalışıyor!");
}

void loop() {
    Serial.println("Merhaba, ESP32C3!");
    delay(1000);
}
