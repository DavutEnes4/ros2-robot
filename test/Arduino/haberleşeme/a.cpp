#include <iostream>
#include <SerialTransfer.h>
#include <cstring>
#include <unistd.h>

struct __attribute__((packed)) STRUCT
{
    char z;
    float y;
} testStruct;

char arr[6];

int main()
{
    serial::Serial mySerial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000)); // Linux'ta "/dev/ttyUSB0"

    if (!mySerial.isOpen())
    {
        std::cerr << "Seri port açılamadı!" << std::endl;
        return -1;
    }

    std::cout << "Seri port açıldı!" << std::endl;

    while (true)
    {
        if (mySerial.available())
        {
            // Veriyi al
            mySerial.read(reinterpret_cast<uint8_t *>(&testStruct), sizeof(testStruct));
            std::cout << testStruct.z << " | " << testStruct.y << " | ";

            // Karakter dizisini al
            mySerial.read(reinterpret_cast<uint8_t *>(arr), sizeof(arr));
            std::cout << arr << std::endl;
        }
        usleep(1000); // 1ms bekle
    }

    return 0;
}
