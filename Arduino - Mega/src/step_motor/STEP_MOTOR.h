#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <Arduino.h>
#include <Stepper.h>
#include <ArduinoJson.h>

class step_motor {
private:
    int stepsPerRevolution;  // Motorun her devrinde adım sayısı
    Stepper motor;           // Stepper motor nesnesi

    int enable_pin;
    int dir_pin;
    int pwm_pin;
    unsigned long lastStepTime;
    unsigned long step_interval; // Adım interval süresi

public:
    step_motor(int enable_pin, int dir_pin, int pwm_pin, int stepsPerRevolution);
    void step();
    void set_speed_rpm(long whatSpeed);
    void set_direction(bool direction);
    void enable();
    void disable();
    void info();
    String jsonWrite();
};

#endif
