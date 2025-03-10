// StepperController.h
#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include <Arduino.h>

#define MAX_SPEED 250  // PWM için maksimum hız (0-255)

class StepperController {
private:
    int enablePin1, dirPin1, pwmPin1;
    int enablePin2, dirPin2, pwmPin2;
    bool rotated; // 45 derece döndürülmüş eksen mi?

public:
    StepperController(int enable1, int dir1, int pwm1, int enable2, int dir2, int pwm2, bool isRotated = false);
    void move(double x, double y);
};

#endif // STEPPER_CONTROLLER_H
