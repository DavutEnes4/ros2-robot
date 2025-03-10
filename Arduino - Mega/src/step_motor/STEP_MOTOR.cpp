#include <Arduino.h>
#include <Stepper.h>

#define STEPS_PER_REV 200  // Step motorun bir turu için gereken adım sayısı
#define MAX_SPEED 100      // Maksimum hız (RPM cinsinden)

class StepperController {
private:
    Stepper motorX;
    Stepper motorY;
    bool rotated; // 45 derece döndürülmüş eksen mi?

public:
    StepperController(int xPin1, int xPin2, int xPin3, int xPin4,
                      int yPin1, int yPin2, int yPin3, int yPin4,
                      bool isRotated = false)
        : motorX(STEPS_PER_REV, xPin1, xPin2, xPin3, xPin4),
          motorY(STEPS_PER_REV, yPin1, yPin2, yPin3, yPin4),
          rotated(isRotated) {}

    void move(double x, double y) {
        int stepX, stepY;
        
        // Hız hesaplama: speed = sqrt(x^2 + y^2) * MAX_SPEED
        double speedFactor = sqrt(x * x + y * y);
        int dynamicSpeed = speedFactor * MAX_SPEED;
        motorX.setSpeed(dynamicSpeed);
        motorY.setSpeed(dynamicSpeed);
        
        if (rotated) {
            // 45 derece döndürülmüş eksene göre dönüşüm
            stepX = (x - y) * STEPS_PER_REV;
            stepY = (x + y) * STEPS_PER_REV;
        } else {
            // Düz eksen
            stepX = x * STEPS_PER_REV;
            stepY = y * STEPS_PER_REV;
        }
        
        motorX.step(stepX);
        motorY.step(stepY);
    }
};
