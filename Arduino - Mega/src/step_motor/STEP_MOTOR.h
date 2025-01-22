/**
 * STEP_MOTOR.h - Adımlı motor kontrolü için tanımlamalar.
 * 
 * Bu sınıf, bir adımlı motorun hızını, yönünü ve çalışmasını kontrol etmek için temel işlevleri içerir.
 * 
 * Özellikler:
 * - Adım süresini RPM (dakikadaki dönüş sayısı) cinsinden ayarlama
 * - Motorun yönünü değiştirme
 * - Motoru etkinleştirme ve devre dışı bırakma
 * - Bir adım hareket ettirme
 */

#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <Arduino.h>

class step_motor
{
public:
    /**
     * @brief Adımlı motor sınıfı oluşturucu
     * 
     * @param enable_pin Motoru etkinleştirme pini
     * @param dir_pin Yön kontrol pini
     * @param pwm_pin PWM sinyali için pin
     */
    step_motor(int enable_pin, int dir_pin, int pwm_pin);

    /**
     * @brief Motoru bir adım hareket ettirir
     */
    void step();

    /**
     * @brief Motorun hızını RPM cinsinden ayarlar
     * 
     * @param rpm Motorun dakika başına dönüş hızı
     */
    void set_speed_rpm(int rpm);

    /**
     * @brief Motorun yönünü belirler
     * 
     * @param direction Yön (true: ileri, false: geri)
     */
    void set_direction(bool direction);

    /**
     * @brief Motoru etkinleştirir
     */
    void enable();

    /**
     * @brief Motoru devre dışı bırakır
     */
    void disable();

private:
    int enable_pin;      /**< Motoru etkinleştirme pini */
    int dir_pin;         /**< Yön kontrol pini */
    int pwm_pin;         /**< PWM sinyali için pin */
    int step_period;     /**< Adım süresi (mikro saniye) */
};

#endif // STEP_MOTOR_H
