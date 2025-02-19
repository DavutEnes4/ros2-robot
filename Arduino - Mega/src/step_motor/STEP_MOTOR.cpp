#include "STEP_MOTOR.h"
#include <Arduino.h>
#include <ArduinoJson.h>

step_motor::step_motor(int enable_pin, int dir_pin, int pwm_pin)
{
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->step_period = 1;
    this->maxRadius = 1;
    this->maxFrequency = 1;
    this->stepInterval = 0;
    this->lastStepTime = 0;

    pinMode(enable_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    digitalWrite(enable_pin, HIGH); // Motoru başlangıçta durdur
}

step_motor::step_motor(int enable_pin, int dir_pin, int pwm_pin, double maxRadius, double maxFrequency)
{
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->step_period = 1;
    this->maxRadius = maxRadius;
    this->maxFrequency = maxFrequency;
    this->stepInterval = 0;
    this->lastStepTime = 0;

    pinMode(enable_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    digitalWrite(enable_pin, HIGH); // Motoru başlangıçta durdur
}


void step_motor::step()
{
    analogWrite(pwm_pin, step_period);
}

void step_motor::stepFrequency(double r)
{
    if (r < 0) r = 0; // r'nin minimum değeri
        if (r > maxRadius) r = maxRadius; // r'nin maksimum değeri

        // r'ye göre frekans hesapla (lineer ölçekleme)
        double frequency = (r / maxRadius) * maxFrequency;

        // Frekansı adım süresine dönüştür
        stepInterval = (frequency > 0) ? 1000000.0 / frequency : 0;

        // Adımları zamanlama ile kontrol et
        unsigned long currentTime = micros();
        if (currentTime - lastStepTime >= stepInterval && stepInterval > 0) {
            // PWM sinyalini bir adım için değiştir
            digitalWrite(pwm_pin, HIGH);
            delayMicroseconds(stepInterval / 2); // Hareket süresinin yarısı kadar bekle
            digitalWrite(pwm_pin, LOW);
            delayMicroseconds(stepInterval / 2);

            // Son adım zamanını güncelle
            lastStepTime = currentTime;
        }
}

void step_motor::set_direction(bool direction)
{
    digitalWrite(dir_pin, direction ? HIGH : LOW); // Yönü ayarla
}

void step_motor::set_speed_rpm(int rpm)
{
    this->step_period = rpm%256; // 0-255 arasında bir değer olmalı
}

void step_motor::enable()
{
    digitalWrite(enable_pin, LOW); // Motoru aktif et
}

void step_motor::disable()
{
    digitalWrite(enable_pin, HIGH); // Motoru devre dışı bırak
}

void step_motor::info()
{
    Serial.println("Motor Configuration:");
    Serial.println("Enable Pin: " + String(enable_pin));
    Serial.println("Direction Pin: " + String(dir_pin));
    Serial.println("PWM Pin: " + String(pwm_pin));
    Serial.println("Step Period: " + String(step_period) + " microseconds");
    Serial.println("Max Radius: " + String(maxRadius));
    Serial.println("Max Frequency: " + String(maxFrequency) + " Hz");
    Serial.println("Last Step Time: " + String(lastStepTime));
    Serial.println("Step Interval: " + String(stepInterval) + " microseconds");
}

String step_motor::jsonWrite()
{
    StaticJsonDocument<256> jsonDoc;

    jsonDoc["enable_pin"] = enable_pin;
    jsonDoc["dir_pin"] = dir_pin;
    jsonDoc["pwm_pin"] = pwm_pin;
    jsonDoc["step_period"] = step_period;
    jsonDoc["maxRadius"] = maxRadius;
    jsonDoc["maxFrequency"] = maxFrequency;
    jsonDoc["lastStepTime"] = lastStepTime;
    jsonDoc["stepInterval"] = stepInterval;

    String jsonStr;
    serializeJson(jsonDoc, jsonStr);
    return jsonStr;
}