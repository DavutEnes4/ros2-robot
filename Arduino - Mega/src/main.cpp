#include <Arduino.h>
#include <mpu9250/MPU9250.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define CALIBRATE_COUNT_MPU 1000
#define MPU9250_ADDR 0x68

// Sağ motor bağlantısı
#define rightEnable 23
#define rightDirPin 25
#define rightPwm 2

// Sol motor bağlantısı
#define leftEnable 22
#define leftDirPin 24
#define leftPwm 3

AccelStepper stepperRight(1, rightDirPin, rightPwm);
AccelStepper stepperLeft(1, leftDirPin, leftPwm);

MultiStepper steppers;

mpu9250 mpu(MPU9250_ADDR, CALIBRATE_COUNT_MPU);

double maxSpeed = 10000;
double inputX;
double inputY;

void fSerialRead();
void fSerialBuffer();
String fSensorRead();
void fSerialWrite();
void fStepperMove();

void setup()
{
  Serial.begin(115200);

  pinMode(rightEnable, OUTPUT);
  pinMode(leftEnable, OUTPUT);

  digitalWrite(rightEnable, LOW);
  digitalWrite(leftEnable, LOW);
  Wire.begin();
  mpu.setup();
  mpu.calibrate();
  stepperRight.setMaxSpeed(maxSpeed);
  stepperLeft.setMaxSpeed(maxSpeed);
  stepperRight.setAcceleration(maxSpeed/2);
  stepperLeft.setAcceleration(maxSpeed/2);
  steppers.addStepper(stepperRight);
  steppers.addStepper(stepperLeft);
}

void loop()
{
  fSerialWrite();
  fSerialRead();
  fStepperMove();
}

void fSerialRead() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    int index = data.indexOf(',');
    if (index == -1) return;
    
    inputX = data.substring(0, index).toDouble();
    inputY = data.substring(index + 1).toDouble();
    Serial.print("Gelen değer:");
    Serial.print(inputY);
    Serial.print(",");
    Serial.println(inputX);
  }
}


void fSerialBuffer()
{
  if (Serial.available())
  {
    char buffer[32];
    int length = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[length] = '\0';
    String data = String(buffer);
    data.trim();
    int index = data.indexOf(',');
    // raw_vector[0] = data.substring(0, index).toDouble();
    // raw_vector[1] = data.substring(index + 1).toDouble();
    inputX = data.substring(0, index).toDouble();
    inputY = data.substring(index + 1).toDouble();
  }
}

String fSensorRead()
{
  StaticJsonDocument<1024> jsonDoc;
  String strMpu = mpu.jsonWrite();
  jsonDoc["mpu9250"] = strMpu;
  jsonDoc["time"] = String(millis());
  char jsonBuffer[1024];
  serializeJson(jsonDoc, jsonBuffer);
  return String(jsonBuffer);
}

void fSerialWrite()
{
  String jsonString = fSensorRead();
  Serial.println(jsonString);
}

// void fStepperMove()
// {
//   // if (inputY < 0.02 && inputY > -0.02)
//   //   return;

//   long leftSpeed = (inputY + inputX / 2) * maxSpeed;
//   long rightSpeed = (inputY - inputX / 2) * maxSpeed;

//   // Hız değerlerini sınırlandır
//   leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
//   rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

//   long positions[2] = {leftSpeed, rightSpeed};
//   steppers.moveTo(positions);
//   steppers.runSpeedToPosition();
// }

void fStepperMove() {
  if (inputY < 0.02 && inputY > -0.02)
    return;

  long leftSpeed = (inputY + inputX / 2) * maxSpeed;
  long rightSpeed = (inputY - inputX / 2) * maxSpeed;

  // Hız değerlerini sınırlandır
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  stepperLeft.setSpeed(leftSpeed);
  stepperRight.setSpeed(rightSpeed);

  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}