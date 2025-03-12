#include <Wire.h>
#include <TinyGPSPlus.h>
#include <MPU9250_WE.h>

// Pin tanımlamaları
const int rightEnablePin = 23;
const int rightDirPin = 25;
const int rightPwmPin = 2;
const int leftEnablePin = 22;
const int leftDirPin = 24;
const int leftPwmPin = 3;

// Hareket modu sabitleri
#define MODE_DIRECT 0   // Doğrudan açısal hareket
#define MODE_CAR_LIKE 1 // Araba benzeri hareket

// Koordinat sistemi sabitleri
#define COORD_NORMAL 0  // Normal düzlem
#define COORD_ROTATED 1 // 45 derece çevrilmiş düzlem

// Global değişkenler
int moveMode = MODE_DIRECT;
int coordSystem = COORD_NORMAL;
float currentHeading = 0.0; // Derece cinsinden mevcut yön

// Sensör nesneleri
TinyGPSPlus gps;
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

// ROS2 ile haberleşme için buffer
String inputBuffer = "";
bool commandComplete = false;

void printHelp();
void updateHeading();
void processCommands();
void executeCommand(String command);
void navigateToLocation(float targetLat, float targetLon, int speed);
void moveRobot(float x, float y, int speed);
void setMotorSpeed(bool isLeft, int speed);
void stopMotors();
void printStatus();
void printHelp();

void setup()
{
    // Ana seri port (PC ile iletişim için)
    Serial.begin(115200);

    // GPS seri portu
    Serial1.begin(9600);

    // Motor pinlerini ayarla
    pinMode(rightEnablePin, OUTPUT);
    pinMode(rightDirPin, OUTPUT);
    pinMode(rightPwmPin, OUTPUT);
    pinMode(leftEnablePin, OUTPUT);
    pinMode(leftDirPin, OUTPUT);
    pinMode(leftPwmPin, OUTPUT);

    // Motorları aktif et
    digitalWrite(rightEnablePin, LOW);
    digitalWrite(leftEnablePin, LOW);

    // I2C iletişimi başlat
    Wire.begin();
    delay(2000);

    // MPU9250 başlat
    if (!myMPU9250.init())
    {
        Serial.println("MPU9250 bağlantı hatası!");
    }
    else
    {
        Serial.println("MPU9250 başlatıldı");
    }

    // MPU9250 ayarları
    myMPU9250.autoOffsets();
    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.setSampleRateDivider(5);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

    Serial.println("Robot kontrol sistemi başlatıldı");
    printHelp();
}

void loop()
{
    // GPS verisini güncelle
    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
    }

    // MPU verisini güncelle ve mevcut yönü hesapla
    updateHeading();

    // Kullanıcı komutlarını işle
    processCommands();

    // Hareket ve diğer fonksiyonlar burada çağrılacak
}

void updateHeading()
{
    // MPU9250'den veri al
    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();
    xyzFloat magValue = myMPU9250.getMagValues();

    // Manyetometre ile pusula yönünü hesapla
    float heading = atan2(magValue.y, magValue.x) * 180 / PI;

    // Manyetometre değerini düzelt (negatif değerleri 0-360 aralığına getir)
    if (heading < 0)
    {
        heading += 360;
    }

    // Basit bir alçak geçirgen filtre uygula
    static float filteredHeading = heading;
    filteredHeading = 0.9 * filteredHeading + 0.1 * heading;

    // Filtrelenmiş değeri kaydet
    currentHeading = filteredHeading;
}

void processCommands()
{
    // Seri porttan komutları oku
    while (Serial.available() > 0 && !commandComplete)
    {
        char inChar = (char)Serial.read();
        if (inChar == '\n')
        {
            commandComplete = true;
        }
        else
        {
            inputBuffer += inChar;
        }
    }

    // Komut tamamlandıysa işle
    if (commandComplete)
    {
        executeCommand(inputBuffer);
        inputBuffer = "";
        commandComplete = false;
    }
}

void executeCommand(String command)
{
    command.trim();

    if (command.startsWith("MOVE"))
    {
        // MOVE X,Y,SPEED komutu (X,Y: -100 ile 100 arası, SPEED: 0-100 arası)
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0)
        {
            float x = command.substring(5, firstComma).toFloat();
            float y = command.substring(firstComma + 1, secondComma).toFloat();
            int speed = command.substring(secondComma + 1).toInt();

            // Koordinat sistemine göre dönüşüm yap
            if (coordSystem == COORD_ROTATED)
            {
                // 45 derece çevrilmiş koordinat sisteminden normal sisteme dönüştür
                float tempX = x;
                float tempY = y;
                x = 0.7071 * (tempX - tempY); // cos(45) = 0.7071
                y = 0.7071 * (tempX + tempY); // sin(45) = 0.7071
            }

            // Hareket moduna göre motoru kontrol et
            moveRobot(x, y, speed);
        }
    }
    else if (command == "MODE DIRECT")
    {
        moveMode = MODE_DIRECT;
        Serial.println("Mod: Doğrudan açısal hareket");
    }
    else if (command == "MODE CAR")
    {
        moveMode = MODE_CAR_LIKE;
        Serial.println("Mod: Araba benzeri hareket");
    }
    else if (command == "COORD NORMAL")
    {
        coordSystem = COORD_NORMAL;
        Serial.println("Koordinat sistemi: Normal");
    }
    else if (command == "COORD ROTATED")
    {
        coordSystem = COORD_ROTATED;
        Serial.println("Koordinat sistemi: 45 derece çevrilmiş");
    }
    else if (command == "STATUS")
    {
        printStatus();
    }
    else if (command == "HELP")
    {
        printHelp();
    }
    else if (command.startsWith("GOTO"))
    {
        // GOTO LAT,LON,SPEED komutu - belirli bir GPS konumuna git
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0)
        {
            float targetLat = command.substring(5, firstComma).toFloat();
            float targetLon = command.substring(firstComma + 1, secondComma).toFloat();
            int speed = command.substring(secondComma + 1).toInt();

            // Hedef konuma doğru yönelme (basit algoritma)
            if (gps.location.isValid())
            {
                navigateToLocation(targetLat, targetLon, speed);
            }
            else
            {
                Serial.println("GPS verisi geçersiz, hedef konuma gidemiyorum");
            }
        }
    }
    else
    {
        Serial.println("Bilinmeyen komut. HELP yazarak komutları görüntüleyebilirsiniz.");
    }
}

void navigateToLocation(float targetLat, float targetLon, int speed)
{
    // Mevcut konum
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();

    // Hedefe olan yönü hesapla (basit haversine formülü)
    float dLon = (targetLon - currentLon) * PI / 180.0;
    float dLat = (targetLat - currentLat) * PI / 180.0;

    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(currentLat * PI / 180.0) * cos(targetLat * PI / 180.0) *
                  sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = 6371000 * c; // Metre cinsinden mesafe

    // Hedefin açısını hesapla
    float y = sin(dLon) * cos(targetLat * PI / 180.0);
    float x = cos(currentLat * PI / 180.0) * sin(targetLat * PI / 180.0) -
              sin(currentLat * PI / 180.0) * cos(targetLat * PI / 180.0) * cos(dLon);
    float bearing = atan2(y, x) * 180 / PI;
    if (bearing < 0)
    {
        bearing += 360;
    }

    Serial.print("Hedef: ");
    Serial.print(targetLat, 6);
    Serial.print(", ");
    Serial.print(targetLon, 6);
    Serial.print(" Mesafe: ");
    Serial.print(distance);
    Serial.print("m Açı: ");
    Serial.println(bearing);

    // Hedef açı ile mevcut açı arasındaki farkı hesapla
    float angleDiff = bearing - currentHeading;
    if (angleDiff > 180)
        angleDiff -= 360;
    if (angleDiff < -180)
        angleDiff += 360;

    // Hedefe doğru hareketi başlat
    // Basit bir P kontrol: açı farkı büyükse daha fazla dönüş
    float turnFactor = angleDiff / 45.0; // -1 ile 1 arasında normalize et
    turnFactor = constrain(turnFactor, -1, 1);

    // Mesafeye göre ileri hızı ayarla
    float forwardSpeed = 100; // Varsayılan tam hız
    if (distance < 10)
    {
        forwardSpeed = 30; // Yaklaştıkça yavaşla
    }
    else if (distance < 5)
    {
        forwardSpeed = 20;
    }
    else if (distance < 2)
    {
        forwardSpeed = 10;
    }

    // Hareket komutlarını oluştur
    x = forwardSpeed;
    y = turnFactor * 100; // Dönüş miktarı

    moveRobot(x, y, speed);
}


void moveRobot(float x, float y, int speed)
{
    // x, y: -100 ile 100 arası normalize edilmiş hareket vektörü
    // speed: 0-100 arası hız limiti

    // Hızı 0-255 aralığına ölçekle
    speed = map(speed, 0, 100, 0, 255);

    // Hareket vektörünü normalize et (eğer gerekiyorsa)
    float magnitude = sqrt(x * x + y * y);
    if (magnitude > 100)
    {
        x = (x / magnitude) * 100;
        y = (y / magnitude) * 100;
    }

    // Vektörün açısını hesapla
    float angle = atan2(y, x) * 180 / PI; // Derece cinsinden

    // Eğer vektör çok küçükse, hareketi durdur
    if (magnitude < 5)
    {
        stopMotors();
        return;
    }

    // Hareket moduna göre motor hızlarını hesapla
    float leftSpeed, rightSpeed;

    if (moveMode == MODE_DIRECT)
    {
        // Doğrudan açısal hareket modu (differansiyel sürüş)
        // x: ileri/geri, y: sağa/sola dönüş
        leftSpeed = (x - y) * speed / 100.0;
        rightSpeed = (x + y) * speed / 100.0;
    }
    else
    { // MODE_CAR_LIKE
        // Araba benzeri hareket
        // Diferansiyel sürüş ile araba benzeri hareket simüle edilir
        // Sağa/sola dönüş açısını hesapla
        float turnAngle = atan2(x, y) * 180 / PI; // Derece cinsinden

        // İleri/geri yönü belirle
        bool forward = (y >= 0);

        // Dönüş açısına göre motor hızlarını ayarla
        float turnRatio = abs(turnAngle) / 90.0; // 0-1 arası
        if (turnRatio > 1)
            turnRatio = 1;

        if (turnAngle > 0)
        { // Sağa dönüş
            leftSpeed = forward ? speed : -speed;
            rightSpeed = forward ? speed * (1 - turnRatio) : -speed * (1 - turnRatio);
        }
        else
        { // Sola dönüş
            leftSpeed = forward ? speed * (1 - turnRatio) : -speed * (1 - turnRatio);
            rightSpeed = forward ? speed : -speed;
        }
    }

    // Motor hızlarını sınırla ve uygula
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    setMotorSpeed(true, leftSpeed);   // Sol motor
    setMotorSpeed(false, rightSpeed); // Sağ motor

    Serial.print("Hareket: X=");
    Serial.print(x);
    Serial.print(", Y=");
    Serial.print(y);
    Serial.print(", Hız=");
    Serial.print(speed);
    Serial.print(", Sol=");
    Serial.print(leftSpeed);
    Serial.print(", Sağ=");
    Serial.println(rightSpeed);
}

void setMotorSpeed(bool isLeft, int speed)
{
    // İlgili motor pinlerini seç
    int enablePin = isLeft ? leftEnablePin : rightEnablePin;
    int dirPin = isLeft ? leftDirPin : rightDirPin;
    int pwmPin = isLeft ? leftPwmPin : rightPwmPin;

    // Yön ve hızı ayarla
    if (speed == 0)
    {
        digitalWrite(enablePin, HIGH); // Motoru devre dışı bırak
    }
    else
    {
        if (isLeft)
            speed = -speed;            // Mutlak değeri al
            
        digitalWrite(enablePin, LOW); // Motoru etkinleştir
        if (speed > 0)
        {
            digitalWrite(dirPin, HIGH); // İleri
        }
        else
        {
            digitalWrite(dirPin, LOW); // Geri
            speed = -speed;            // Mutlak değeri al
        }
        analogWrite(pwmPin, speed); // PWM hızını ayarla
    }
}

void stopMotors()
{
    setMotorSpeed(true, 0);  // Sol motor
    setMotorSpeed(false, 0); // Sağ motor
    Serial.println("Motorlar durduruldu");
}

void printStatus()
{
    Serial.println("------- Robot Durumu -------");
    Serial.print("Hareket modu: ");
    Serial.println(moveMode == MODE_DIRECT ? "Doğrudan" : "Araba benzeri");
    Serial.print("Koordinat sistemi: ");
    Serial.println(coordSystem == COORD_NORMAL ? "Normal" : "Çevrilmiş");
    Serial.print("Mevcut yön: ");
    Serial.println(currentHeading);

    // GPS durumunu yazdır
    if (gps.location.isValid())
    {
        Serial.print("GPS Konum: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Hız: ");
        Serial.print(gps.speed.kmph());
        Serial.println(" km/s");
        Serial.print("Yükseklik: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");
        Serial.print("Uydu sayısı: ");
        Serial.println(gps.satellites.value());
    }
    else
    {
        Serial.println("GPS Konum: Geçersiz");
    }

    // MPU durumunu yazdır
    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();
    xyzFloat magValue = myMPU9250.getMagValues();

    Serial.print("İvme (g): x = ");
    Serial.print(gValue.x);
    Serial.print(", y = ");
    Serial.print(gValue.y);
    Serial.print(", z = ");
    Serial.println(gValue.z);

    Serial.print("Jiroskop (°/s): x = ");
    Serial.print(gyr.x);
    Serial.print(", y = ");
    Serial.print(gyr.y);
    Serial.print(", z = ");
    Serial.println(gyr.z);

    Serial.print("Manyetometre (µT): x = ");
    Serial.print(magValue.x);
    Serial.print(", y = ");
    Serial.print(magValue.y);
    Serial.print(", z = ");
    Serial.println(magValue.z);

    Serial.println("----------------------------");
}

void printHelp()
{
    Serial.println("----- Robot Kontrol Komutları -----");
    Serial.println("MOVE X,Y,SPEED - X,Y: -100 ile 100 arası, SPEED: 0-100 arası");
    Serial.println("GOTO LAT,LON,SPEED - Belirtilen GPS konumuna git");
    Serial.println("MODE DIRECT - Doğrudan açısal hareket moduna geç");
    Serial.println("MODE CAR - Araba benzeri hareket moduna geç");
    Serial.println("COORD NORMAL - Normal koordinat sistemini kullan");
    Serial.println("COORD ROTATED - 45 derece çevrilmiş koordinat sistemini kullan");
    Serial.println("STATUS - Robot durumunu görüntüle");
    Serial.println("HELP - Bu yardım mesajını görüntüle");
    Serial.println("---------------------------------");
}