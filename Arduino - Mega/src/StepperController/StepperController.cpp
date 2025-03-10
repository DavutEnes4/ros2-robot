// StepperController.cpp
#include "StepperController.h"

StepperController::StepperController(int enable1, int dir1, int pwm1, int enable2, int dir2, int pwm2, bool isRotated)
    : enablePin1(enable1), dirPin1(dir1), pwmPin1(pwm1),
      enablePin2(enable2), dirPin2(dir2), pwmPin2(pwm2), rotated(isRotated)
{
    pinMode(enablePin1, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(pwmPin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(pwmPin2, OUTPUT);
    digitalWrite(enablePin1, LOW); // Motor 1 aktif
    digitalWrite(enablePin2, LOW); // Motor 2 aktif
}

void StepperController::move(double x, double y)
{
    
    if (x > -0.05 && x < 0.05)
        x = 0;

    if (y > -0.05 && y < 0.05)
        y = 0;

    // Hız hesaplama: speed = sqrt(x^2 + y^2) * MAX_SPEED
    double speedFactor = sqrt(x * x + y * y);

    // Eğer joystick hareketsizse motorları durdur
    if (speedFactor < 0.05)
    { // Küçük hareketleri filtrelemek için eşik değeri
        analogWrite(pwmPin1, 0);
        analogWrite(pwmPin2, 0);
        return;
    }

    int Speed = speedFactor * MAX_SPEED;
    int pwmSpeed1 = abs((x / speedFactor) * Speed);
    int pwmSpeed2 = abs((y / speedFactor) * Speed);

    // Minimum hız 20 olsun ki motor çalışsın
    pwmSpeed1 = constrain(pwmSpeed1, 20, MAX_SPEED);
    pwmSpeed2 = constrain(pwmSpeed2, 20, MAX_SPEED);

    // 45 derece döndürülmüş eksene göre dönüşüm
    double motionX = rotated ? (x - y) : x;
    double motionY = rotated ? (x + y) : y;

    // Yön belirleme ve motor kontrolü
    digitalWrite(dirPin1, motionX >= 0 ? HIGH : LOW);
    digitalWrite(dirPin2, motionY >= 0 ? HIGH : LOW);

    // Hızları uygula
    analogWrite(pwmPin1, pwmSpeed1);
    analogWrite(pwmPin2, pwmSpeed2);

}
