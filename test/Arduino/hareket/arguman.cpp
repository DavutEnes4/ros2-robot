#include <iostream>
#include <cmath>  // math.h yerine cmath kullanıldı

using namespace std;

#define PI 3.141592653589793  // PI tanımlandı

double raw_vector[2] = {0, 0};
double converted_vector[2] = {0, 0};

int main(int argc, char *argv[]) {
    if (argc < 3) {  // En az iki argüman kontrolü
        cerr << "Hata: Lütfen iki sayı giriniz." << endl;
        return 1;
    }

     raw_vector[0] = stod(argv[1]);  // String'i double'a çevirme
     raw_vector[1] = stod(argv[2]) / 2;

    double r = sqrt(pow(raw_vector[0], 2) + pow(raw_vector[1], 2));  // sq yerine pow(x,2) kullanıldı
    double theta = atan2(raw_vector[1], raw_vector[0]);

    converted_vector[0] = r * cos(theta + PI/4);
    converted_vector[1] = r * sin(theta + PI/4);

    double right_speed = raw_vector[0];
    double left_speed = raw_vector[0];
    if(raw_vector[0]){
        right_speed -= raw_vector[1];
        left_speed += raw_vector[1];
    }
    cout << "R: " << r << " Theta: " << theta << endl;
    cout << "Y Vector: " << raw_vector[0] << " X Vector: " << raw_vector[1] << endl;
    cout << "Right Speed: " << right_speed << " Left Speed: " << left_speed << endl;
    cout << "Convert Right Speed: " << converted_vector[0] + converted_vector[1] << " Convert Left Speed: " <<converted_vector[0] - converted_vector[1] << endl;

    return 0;
}
