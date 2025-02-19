#include "MPU9250.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Constructor
mpu9250::mpu9250(uint8_t addr, int kalibrate_count_mpu)
    : addr(addr), kalibrate_count_mpu(kalibrate_count_mpu), mpu(addr) {}

// Destructor
mpu9250::~mpu9250() {
    // Dinamik bellek kullansaydınız, burada serbest bırakma işlemi yapılırdı.
}

// MPU9250'yi Başlat ve Ayarlarını Yap
void mpu9250::setup() {
    if (!mpu.init()) {
        Serial.println("MPU9250 başlatılamadı!");
        while (1); // Başarısızsa dur
    }

    if (!mpu.initMagnetometer()) {
        Serial.println("Manyetometre başlatılamadı!");
        while (1); // Başarısızsa dur
    }

    // MPU9250 Parametre Ayarları
    delay(200); // İlk stabilizasyon için kısa bir gecikme
    mpu.enableGyrDLPF();// Jiroskop için DLPF filtresini etkinleştir
    mpu.setGyrDLPF(MPU9250_DLPF_2); // Jiroskop için DLPF filtresini ayarla
    mpu.setSampleRateDivider(5); // Örnekleme oranını ayarla
    mpu.setGyrRange(MPU9250_GYRO_RANGE_250); // Jiroskop hassasiyetini ayarla
    mpu.setAccRange(MPU9250_ACC_RANGE_2G); // İvmeölçer hassasiyetini ayarla
    mpu.enableAccDLPF(true); // İvmeölçer için DLPF filtresini etkinleştir
    mpu.setAccDLPF(MPU9250_DLPF_2); // İvmeölçer için DLPF filtresini ayarla
    mpu.enableAccAxes(MPU9250_ENABLE_XYZ); // İvmeölçer eksenlerini etkinleştir
    mpu.enableGyrAxes(MPU9250_ENABLE_XYZ); // Jiroskop eksenlerini etkinleştir
    mpu.setMagOpMode(AK8963_CONT_MODE_100HZ); // Manyetometre çalışma modunu ayarla
}

// MPU9250'yi Kalibre Et
void mpu9250::calibrate() {
    xyzFloat accelSum = {0.f, 0.f, 0.f};
    xyzFloat gyroSum = {0.f, 0.f, 0.f};

    for (int i = 0; i < kalibrate_count_mpu; i++) {
        accelSum += mpu.getAccRawValues();
        gyroSum += mpu.getGyrRawValues();
        delay(1);
    }

    accelSum /= kalibrate_count_mpu;
    accelSum.z -= 16384.0f; // Yerçekimi etkisini çıkar
    mpu.setAccOffsets(accelSum);
    mpu.setGyrOffsets(gyroSum / kalibrate_count_mpu);
}

// MPU9250'den Verileri Okur
void mpu9250::read() {
    accel = mpu.getGValues();
    gyro = mpu.getGyrValues();
    mag = mpu.getMagValues();
    temp = mpu.getTemperature();
    resultantG = mpu.getResultantG(accel);
}

// MPU9250'den Veri Oku ve JSON Formatında Döndür
String mpu9250::jsonWrite() {
    read();
    StaticJsonDocument<256> jsonDoc;

    jsonDoc["accel"]["x"] = accel.x;
    jsonDoc["accel"]["y"] = accel.y;
    jsonDoc["accel"]["z"] = accel.z;

    jsonDoc["gyro"]["x"] = gyro.x;
    jsonDoc["gyro"]["y"] = gyro.y;
    jsonDoc["gyro"]["z"] = gyro.z;

    jsonDoc["mag"]["x"] = mag.x;
    jsonDoc["mag"]["y"] = mag.y;

    jsonDoc["temp"] = temp;
    jsonDoc["resultantG"] = resultantG;

    String jsonStr;
    serializeJson(jsonDoc, jsonStr);
    return jsonStr;
}
