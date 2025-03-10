# Tello Drone ROS 2 Kullanım Dokümanı

Bu doküman, ROS 2 Humble üzerinde Tello drone'unuzu kontrol etmek için geliştirilen `tello_ros2` paketinin kullanımını ve yapısını açıklamaktadır.

## İçindekiler

1. [Genel Bakış](#genel-bakış)
2. [Kurulum](#kurulum)
3. [ROS 2 Düğümleri](#ros-2-düğümleri)
4. [Topicler](#topicler)
5. [Temel Hareketler](#temel-hareketler)
6. [Drone Durum Bilgileri](#drone-durum-bilgileri)
7. [Video Akışı](#video-akışı)
8. [Örnek Kullanım Senaryoları](#örnek-kullanım-senaryoları)
9. [Sorun Giderme](#sorun-giderme)

## Genel Bakış

`tello_ros2` paketi, DJI Tello drone'larını ROS 2 Humble ortamında kontrol etmenizi sağlar. Bu paket şu özellikleri sağlar:

- Temel uçuş komutları (kalkış, iniş, hareket)
- Gelişmiş uçuş komutları (takla atma)
- Drone durum bilgisi takibi
- Canlı video akışı
- Pil durumu izleme

## Kurulum

### Ön Gereksinimler

- ROS 2 Humble
- Python 3.8+
- OpenCV
- NumPy

### Kurulum Adımları

```bash
# Workspace oluşturun (eğer yoksa)
mkdir -p ~/tello_ws/src
cd ~/tello_ws/src

# Paketi oluşturun
ros2 pkg create --build-type ament_python --node-name tello_node tello_ros2

# Paket kaynak dosyalarını kopyalayın veya düzenleyin
# (tello_node.py, setup.py, package.xml vb.)

# Derleme
cd ~/tello_ws
colcon build --packages-select tello_ros2
source install/setup.bash
```

## ROS 2 Düğümleri

### tello_node

Ana kontrol düğümü, drone ile iletişim kurarak komutları iletir ve durum bilgilerini alır.

**Başlatma:**
```bash
ros2 launch tello_ros2 tello_node.launch.py
```

**İşlevler:**
- Drone ile UDP bağlantısı kurma
- Komutları iletme
- Durum bilgilerini yayınlama
- Video akışını işleme

## Topicler

### Yayınlanan Topicler (Publishers)

| Topic Adı | Mesaj Tipi | Açıklama |
|-----------|------------|----------|
| `/tello/state` | `std_msgs/String` | Drone'un anlık durum bilgileri |
| `/tello/image_raw` | `sensor_msgs/Image` | Drone kamerasından gelen görüntü akışı |
| `/tello/battery` | `sensor_msgs/BatteryState` | Pil durumu bilgisi |

### Abone Olunan Topicler (Subscribers)

| Topic Adı | Mesaj Tipi | Açıklama |
|-----------|------------|----------|
| `/tello/cmd_vel` | `geometry_msgs/Twist` | Hız komutları |
| `/tello/takeoff` | `std_msgs/Empty` | Kalkış komutu |
| `/tello/land` | `std_msgs/Empty` | İniş komutu |
| `/tello/flip` | `std_msgs/String` | Takla atma komutu |

## Temel Hareketler

### Kalkış

```bash
ros2 topic pub --once /tello/takeoff std_msgs/Empty "{}"
```

### İniş

```bash
ros2 topic pub --once /tello/land std_msgs/Empty "{}"
```

### Hareket Kontrolü

Drone'un hareketi, `/tello/cmd_vel` topiğine `geometry_msgs/Twist` mesajları göndererek kontrol edilir.

```bash
# İleri hareket (1 m/s)
ros2 topic pub --once /tello/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Sağa hareket (1 m/s)
ros2 topic pub --once /tello/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Yukarı hareket (0.5 m/s)
ros2 topic pub --once /tello/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Saat yönünde dönüş (45 derece/s)
ros2 topic pub --once /tello/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}"
```

### Takla Atma

```bash
# Sola takla
ros2 topic pub --once /tello/flip std_msgs/String "{data: 'l'}"

# Sağa takla
ros2 topic pub --once /tello/flip std_msgs/String "{data: 'r'}"

# İleri takla
ros2 topic pub --once /tello/flip std_msgs/String "{data: 'f'}"

# Geri takla
ros2 topic pub --once /tello/flip std_msgs/String "{data: 'b'}"
```

## Drone Durum Bilgileri

Drone'un durum bilgilerini görmek için:

```bash
ros2 topic echo /tello/state
```

Bu komut, drone'un aşağıdaki bilgilerini gösterir:
- Pil seviyesi (bat)
- Uçuş süresi (time)
- Yükseklik (h)
- Hız değerleri (vgx, vgy, vgz)
- Eğim açıları (pitch, roll, yaw)
- Sıcaklık (templ, temph)

## Video Akışı

Drone'dan gelen video akışını görüntülemek için:

```bash
ros2 run rqt_image_view rqt_image_view
```

RQT arayüzünde `/tello/image_raw` topiğini seçerek video akışını görebilirsiniz.

## Örnek Kullanım Senaryoları

### Otonom Kare Uçuşu

Aşağıdaki Python scripti, drone'un otonom olarak kare şeklinde uçmasını sağlar:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

class SquareFlight(Node):
    def __init__(self):
        super().__init__('square_flight')
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/tello/land', 10)
        
    def takeoff(self):
        self.takeoff_pub.publish(Empty())
        time.sleep(5)  # Kalkış için bekle
        
    def land(self):
        self.land_pub.publish(Empty())
        
    def move(self, x, y, z, yaw, duration):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.z = yaw
        
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        
    def fly_square(self):
        # Kalkış
        self.takeoff()
        
        # Kareyi uç (her kenar 1 metre)
        self.move(0.1, 0.0, 0.0, 0.0, 3)  # İleri
        self.move(0.0, 0.1, 0.0, 0.0, 3)  # Sağa
        self.move(-0.1, 0.0, 0.0, 0.0, 3)  # Geri
        self.move(0.0, -0.1, 0.0, 0.0, 3)  # Sola
        
        # İniş
        self.land()

def main(args=None):
    rclpy.init(args=args)
    node = SquareFlight()
    node.fly_square()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Yüz Takibi Örneği

Bu örnekte, drone kamerası üzerinden yüz tespiti yapılarak drone'un tespit edilen yüzü takip etmesi sağlanır:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        self.image_sub = self.create_subscription(
            Image, '/tello/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                # En büyük yüzü seç
                areas = [w * h for (x, y, w, h) in faces]
                max_idx = areas.index(max(areas))
                x, y, w, h = faces[max_idx]
                
                # Yüz merkezini hesapla
                img_center_x = cv_image.shape[1] / 2
                img_center_y = cv_image.shape[0] / 2
                face_center_x = x + w / 2
                face_center_y = y + h / 2
                
                # Hız komutlarını hesapla
                x_vel = 0.0
                y_vel = (img_center_x - face_center_x) / 300  # Yatay hız
                z_vel = (img_center_y - face_center_y) / 300  # Dikey hız
                yaw_vel = 0.0
                
                # Drone'u hareket ettir
                twist = Twist()
                twist.linear.x = x_vel
                twist.linear.y = y_vel
                twist.linear.z = z_vel
                twist.angular.z = yaw_vel
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Hata: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sorun Giderme

### Bağlantı Sorunları

Eğer drone ile bağlantı kuramazsanız:

1. Drone'un Wi-Fi ağına bağlı olduğunuzdan emin olun
2. Ping testi yapın: `ping 192.168.10.1`
3. Firewall ayarlarını kontrol edin, UDP port 8889, 8890 ve 11111'in açık olduğundan emin olun

### Video Akışı Gelmiyor

Video akışı almıyorsanız:

1. `streamon` komutunun gönderildiğinden emin olun
2. OpenCV kütüphanesinin kurulu olduğunu doğrulayın
3. UDP port 11111'in açık olduğunu kontrol edin
4. Drone bataryasının yeterli olduğundan emin olun

### ROS 2 İletişim Sorunları

ROS 2 bileşenleri arasında iletişim sorunu yaşıyorsanız:

1. ROS domain ID ayarlarını kontrol edin
2. Topic isimlerinin doğru olduğundan emin olun
3. `ros2 topic list` komutunu çalıştırarak mevcut topicler listesini görüntüleyin
4. `ros2 topic echo /tello/state` komutunu çalıştırarak verilerin gelip gelmediğini kontrol edin