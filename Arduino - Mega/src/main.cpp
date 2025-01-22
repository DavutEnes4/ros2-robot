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

double raw_vector[2] = {0, 0};
double converted_vector[2] = {0, 0};

void fSerialRead();
void fVectorConvert();
void fMotorSetSpeed();
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
  fVectorConvert();
  fSetMotor();
  fMotorMove();
}

void fSerialRead()
{
  if (Serial.available() > 0)
  {
    String data = Serial.readString();
    int index = data.indexOf(',');
    raw_vector[0] = data.substring(0, index).toDouble();
    raw_vector[1] = data.substring(index + 1).toDouble();
  }
}

void fVectorConvert()
{
  double r = sqrt(sq(raw_vector[0]) + sq(raw_vector[1]));
  double theta = atan2(raw_vector[1], raw_vector[0]);

  converted_vector[0] = r * cos(theta - PI / 4);
  converted_vector[1] = r * sin(theta - PI / 4);
}

void fSetMotor()
{
  double right_speed = converted_vector[0];
  double left_speed = converted_vector[0];

  right_speed -= converted_vector[1];
  left_speed += converted_vector[1];

  motor_right.set_direction(right_speed > 0);
  motor_left.set_direction(left_speed >! 0);

  motor_right.set_speed_rpm(right_speed);
  motor_left.set_speed_rpm(left_speed);
}

void fMotorMove()
{
  motor_right.step();
  motor_left.step();
}