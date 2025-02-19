#include <Arduino.h>

// İvmeölçer ve jiroskop kütüphanesi
#include <mpu9250/MPU9250.h>
#include <Wire.h>

// İvmeölçer ve jiroskop bağlantısı
#define CALIBRATE_COUNT_MPU 1000 // 1000 ölçüm yaparak MPU9250 için kalibrasyon yapılacak
#define MPU9250_ADDR 0x68        // I2C protokolü MPU9250 adresi

// İvmeölçer ve jiroskop nesnesi
mpu9250 mpu(MPU9250_ADDR, CALIBRATE_COUNT_MPU);


/*
  // Seri haberleşme kütüphanesi
  #include <SerialTransfer.h>

  SerialTransfer myTransfer;
*/
// Motor sürücü kütüphanesi
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
void fSerialBuffer();
String fSensorRead();
void fSerialWrite();
void fVectorConvert();
void fSetMotor();
void fMotorMove();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu.setup();
  mpu.calibrate();

  motor_right.enable();
  motor_left.enable();
}

void loop()
{
  //fSensorRead();
  fSerialRead();
  // fSerialBuffer();
  fSerialWrite();
  // fVectorConvert();
  // fSetMotor();
  fMotorMove();
  delay(500);
}

void fSerialRead()
{
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n'); // Satır sonuna kadar oku
    data.trim();

    int index = data.indexOf(',');
    if (index == -1)
      return;
    raw_vector[0] = data.substring(0, index).toDouble();
    raw_vector[1] = data.substring(index + 1).toDouble();
  }
}

void fSerialBuffer()
{
  if (Serial.available())
  {
    char buffer[32]; // Gelen veri için bir buffer
    int length = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[length] = '\0'; // Null-terminate string

    String data = String(buffer);
    data.trim();

    int index = data.indexOf(',');
    raw_vector[0] = data.substring(0, index).toDouble();
    raw_vector[1] = data.substring(index + 1).toDouble();
  }
}

String fSensorRead()
{
  StaticJsonDocument<1024> jsonDoc;
  String strMpu = mpu.jsonWrite();
  String strRightMotor = motor_right.jsonWrite();
  String strLeftMotor =   motor_left.jsonWrite();

  jsonDoc["mpu9250"] = strMpu;
  jsonDoc["motor_right"] = strRightMotor;
  jsonDoc["motor_left"] = strLeftMotor;
  jsonDoc["time"] = String(millis());

  char jsonBuffer[1024];
  serializeJson(jsonDoc, jsonBuffer);
  return String(jsonBuffer);
}

void fSerialWrite()
{
  // Sensör verisini al
  String jsonString = fSensorRead();
  /*
     Serial Transfer kütüphanesi ile seri porttan veri gönderme
     Seriak Transfer kütüphanesi ile seri porttan veri göndermek için
      Bu işlemi yapmak için aşağıdaki kodları kullanabilirsiniz.
      önce veriyi bir buffer'a yazıp, daha sonra bu buffer'ı seri porttan göndermek gerekiyor.

      Veri göndermek için json formatını kullandık. Json foramtını tercih etme sebebimiz struct yapıları arısıdna veri transferi yaparken
        Kullanacağımız programlama dilerinde struct yapısının farklı olması ve bu yüzden veri transferi yaparken sorun yaşamamak için json formatını tercih ettik.
      
      Bu yöntem genellikle iki mikro denetleyici arasında veri transferi yaparken kullanılır.
      Bu yöntem ile veri transferi yaparken veri kaybı yaşanmaz ve veri transferi daha güvenli olur.
      Bu yöntem ile veri transferi yaparken veri boyutu sınırlıdır. Bu yüzden büyük veri transferi yaparken bu yöntemi kullanmamak daha iyi olur.
    
    // String verisini buffer'a ekle
    char jsonBuffer[512];
    jsonString.toCharArray(jsonBuffer, 512);

    // Veriyi seri porttan gönder
    myTransfer.sendDatum(jsonBuffer, 512);
    myTransfer.sendDatum("\n", 1);
  */
 Serial.println(jsonString);

}

void fVectorConvert()
{
  double r = sqrt(sq(raw_vector[0]) + sq(raw_vector[1]));
  double theta = 0; // atan2(raw_vector[1], raw_vector[0]);
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
  motor_left.set_direction(left_speed > !0);

  motor_right.set_speed_rpm(right_speed);
  motor_left.set_speed_rpm(left_speed);
}

void fMotorMove()
{
  fVectorConvert();
  fSetMotor();

  motor_right.step();
  motor_left.step();
}

