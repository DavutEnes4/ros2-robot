#include <Arduino.h>

// Sağ motor bağlantısı
#define motor_right_enable 23
#define motor_right_dir_pin 25
#define motor_right_pwm 2

// Sol motor bağlantısı
#define motor_left_enable 22
#define motor_left_dir_pin 24
#define motor_left_pwm 3

void setup()
{
  // Motor pinleri çıkış olarak ayarlanır
  pinMode(motor_right_enable, OUTPUT);
  pinMode(motor_right_dir_pin, OUTPUT);
  pinMode(motor_right_pwm, OUTPUT);

  pinMode(motor_left_enable, OUTPUT);
  pinMode(motor_left_dir_pin, OUTPUT);
  pinMode(motor_left_pwm, OUTPUT);

  // Motorlar durdurulur
  digitalWrite(motor_right_enable, LOW);
  digitalWrite(motor_right_dir_pin, LOW);
  analogWrite(motor_right_pwm, 0);

  digitalWrite(motor_left_enable, LOW);
  digitalWrite(motor_left_dir_pin, LOW);
  analogWrite(motor_left_pwm, 0);

  // Seri haberleşme başlatılır
  Serial.begin(9600);
}

void loop()
{
  // Motorlar ileri yönde çalıştırılır
  function_motor_start(motor_right_enable, motor_right_dir_pin, motor_right_pwm, 255);
  function_motor_start(motor_left_enable, motor_left_dir_pin, motor_left_pwm, 255);

  // 1 saniye beklenir
  delay(1000);

  // Motorlar durdurulur
  function_motor_stop(motor_right_enable, motor_right_dir_pin, motor_right_pwm);
  function_motor_stop(motor_left_enable, motor_left_dir_pin, motor_left_pwm);

  // 1 saniye beklenir
  delay(1000);

  // Motorlar geri yönde çalıştırılır
  function_motor_start(motor_right_enable, motor_right_dir_pin, motor_right_pwm, 255);
  function_motor_start(motor_left_enable, motor_left_dir_pin, motor_left_pwm, 255);

  // 1 saniye beklenir
  delay(1000);

  // Motorlar durdurulur
  function_motor_stop(motor_right_enable, motor_right_dir_pin, motor_right_pwm);
  function_motor_stop(motor_left_enable, motor_left_dir_pin, motor_left_pwm);

  // 1 saniye beklenir
  delay(1000);
}

void function_motor_start(int motor_enable, int motor_dir_pin, int motor_pwm, int motor_speed)
{
  digitalWrite(motor_enable, HIGH);
  digitalWrite(motor_dir_pin, HIGH);
  analogWrite(motor_pwm, motor_speed);
}

void function_motor_stop(int motor_enable, int motor_dir_pin, int motor_pwm)
{
  digitalWrite(motor_enable, LOW);
  digitalWrite(motor_dir_pin, LOW);
  analogWrite(motor_pwm, 0);
}