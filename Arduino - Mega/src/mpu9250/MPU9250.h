#ifndef MPU9250_H
#define MPU9250_H

#include <MPU9250_WE.h>
#include <ArduinoJson.h>

/**
 * MPU9250 Sınıfı
 * 
 * Bu sınıf, MPU9250 sensörünü kullanarak ivmeölçer, jiroskop ve manyetometre verilerini okur.
 * Ayrıca, sensörün kalibrasyon işlemlerini yapar.
 * 
 */
class mpu9250 {
public:
    mpu9250(uint8_t addr, int kalibrate_count_mpu);
    ~mpu9250(); // Destructor
    void setup();
    void calibrate();
    void read();
    String jsonWrite();

private:
    uint8_t addr;                  // I2C Adresi
    int kalibrate_count_mpu;       // Kalibrasyon Döngüsü Sayısı
    MPU9250_WE mpu;                // MPU9250 Nesnesi

    struct xyzFloat accel;   // İvmeölçer Değer
    struct xyzFloat gyro;    // Jiroskop Değer
    struct xyzFloat mag;      // Manyetometre Değer
    float temp;                    // Sıcaklık Değeri
    float resultantG;              // Sonuç Değeri

};

#endif