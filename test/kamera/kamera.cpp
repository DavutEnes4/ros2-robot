#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    // Kamerayı başlat
    VideoCapture kamera(0, cv::CAP_V4L2);
    if (!kamera.isOpened())
    {
        cerr << "Kamera açılamadı!" << endl;
        return -1;
    }

    // Kamera özelliklerini ayarla
    kamera.set(cv::CAP_PROP_FRAME_WIDTH, 7680);
    kamera.set(cv::CAP_PROP_FRAME_HEIGHT, 4320);
    kamera.set(cv::CAP_PROP_FPS, 25);

    // Video yazıcıyı başlat
    VideoWriter videoWriter("./yeniVideo.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 25, Size(640, 480));

    if (!videoWriter.isOpened())
    {
        cerr << "Video yazıcı açılamadı!" << endl;
        return -1;
    }

    // Çerçeve okuma ve yazma
    Mat kare;
    int sayac = 0;
    while (kamera.read(kare) && sayac++ < 75) // 75 kare çekelim, 3 saniye.
    {
        videoWriter.write(kare); // Okunan kareyi dosyaya yaz
    }

    cout << "Video kaydedildi!" << endl;
    return 0;
}
