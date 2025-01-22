#include "STEP_MOTOR.h"


step_motor::step_motor(int enable_pin, int dir_pin, int pwm_pin)
{
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->pwm_pin = pwm_pin;
    this->step_period = 1;
    
    pinMode(enable_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    digitalWrite(enable_pin, HIGH); // Motoru başlangıçta durdur
}

void step_motor::step()
{
    analogWrite(pwm_pin, step_period);
}

void step_motor::set_direction(bool direction)
{
    digitalWrite(dir_pin, direction ? HIGH : LOW);
}

void step_motor::set_speed_rpm(int rpm)
{
    this->step_period = rpm%256;
}

void step_motor::enable()
{
    digitalWrite(enable_pin, LOW); // Motoru aktif et
}

void step_motor::disable()
{
    digitalWrite(enable_pin, HIGH); // Motoru devre dışı bırak
}
