#include <Arduino.h>
#include <step_motor/STEP_MOTOR.h>

// Sağ motor bağlantısı
#define motor_right_enable 23
#define motor_right_dir_pin 25
#define motor_right_pwm 2

// Sol motor bağlantısı
#define motor_left_enable 22
#define motor_left_dir_pin 24
#define motor_left_pwm 3

step_motor motor_right(motor_right_enable, motor_right_dir_pin, motor_right_pwm);
step_motor motor_left(motor_left_enable, motor_left_dir_pin, motor_left_pwm);

void fSerialRead();
void fMotorMove();

void setup()
{
  Serial.begin(9600);
  motor_right.enable();
  motor_left.enable();
}

void loop()
{
  fSerialRead();
  fMotorMove();
}

void fSerialRead()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    switch (command)
    {
    case 'w':
      motor_right.set_direction(true);
      motor_left.set_direction(true);
      break;
    case 's':
      motor_right.set_direction(false);
      motor_left.set_direction(false);
      break;
    case 'a':
      motor_right.set_direction(true);
      motor_left.set_direction(false);
      break;
    case 'd':
      motor_right.set_direction(false);
      motor_left.set_direction(true);
      break;
    case 'q':
      motor_right.disable();
      motor_left.disable();
      break;
    default:
      break;
    }
  }
}

void fMotorMove()
{
  motor_right.step();
  motor_left.step();
  delay(1);
}
