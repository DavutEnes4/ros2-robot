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
  // Seri haberleşme ile gelen veri okunur
  if (Serial.available() > 0)
  {
    char data = Serial.read();

    // Gelen veriye göre motorlar kontrol edilir
    switch (data)
    {
    case 'F':
      // İleri hareket
      digitalWrite(motor_right_dir_pin, HIGH);
      digitalWrite(motor_left_dir_pin, HIGH);
      analogWrite(motor_right_pwm, 255);
      analogWrite(motor_left_pwm, 255);
      break;

    case 'B':
      // Geri hareket
      digitalWrite(motor_right_dir_pin, LOW);
      digitalWrite(motor_left_dir_pin, LOW);
      analogWrite(motor_right_pwm, 255);
      analogWrite(motor_left_pwm, 255);
      break;

    case 'L':
      // Sol hareket
      digitalWrite(motor_right_dir_pin, HIGH);
      digitalWrite(motor_left_dir_pin, LOW);
      analogWrite(motor_right_pwm, 255);
      analogWrite(motor_left_pwm, 255);
      break;

    case 'R':
      // Sağ hareket
      digitalWrite(motor_right_dir_pin, LOW);
      digitalWrite(motor_left_dir_pin, HIGH);
      analogWrite(motor_right_pwm, 255);
      analogWrite(motor_left_pwm, 255);
      break;

    case 'S':
      // Durma
      digitalWrite(motor_right_enable, LOW);
      digitalWrite(motor_right_dir_pin, LOW);
      analogWrite(motor_right_pwm, 0);

      digitalWrite(motor_left_enable, LOW);
      digitalWrite(motor_left_dir_pin, LOW);
      analogWrite(motor_left_pwm, 0);
      break;
    }
  }
}
