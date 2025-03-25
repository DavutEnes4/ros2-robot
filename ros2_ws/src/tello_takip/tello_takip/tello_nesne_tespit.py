#!/usr/bin/env python3

"""
TelloNesneTespit
===============
ROS 2 tabanlı bu sınıf, Tello drone'u ile görüntü işleme ve nesne takibi gerçekleştirmek için geliştirilmiştir.
YOLO modeli ile nesne tespiti yapar ve drone'u tespit edilen nesneye göre yönlendirir.

Yetenekler:
- Görüntü alımı ve YOLO modeli ile nesne tespiti
- Drone hareket kontrolü (Twist komutları)
- Pil durumu ve uçuş durumu takibi
- Otomatik arama ve iniş algoritması
"""

import rclpy
from rclpy.node import Node
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import time
import cv2

class TelloNesneTespit(Node):
    """
    Tello drone için nesne tespiti ve takip düğümü.
    """

    def __init__(self):
        """
        ROS 2 düğümünü başlatır, gerekli publisher ve subscriber'ları oluşturur,
        YOLO modelini yükler ve başlangıç parametrelerini ayarlar.

        Raises:
            FileNotFoundError: Model dosyası ('best3.pt') bulunamazsa oluşur.
            Exception: Model yüklemesinde oluşabilecek diğer hatalar.
        """
        super().__init__('tello_nesne_tespit')

        # Görüntü boyutu ve ölü bölge
        self.cercevGenislik = 640
        self.cercevYukseklik = 480
        self.oludBolge = 50  # Drone hareket etmeyecek kadar merkezde olan bölge (px)

        # Durum değişkenleri
        self.nesne_bulundu = False
        self.arama_yapiliyor = False
        self.arama_baslangic_zamani = 0

        # YOLO model dosyasını yükle
        self.model = YOLO("/home/encoder/Desktop/ros2-robot/ros2_ws/src/tello_takip/tello_takip/best3.pt")

        # CV bridge (ROS <-> OpenCV)
        self.kopru = CvBridge()

        # ROS publisher'ları
        self.hiz_komutu_yayinci = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.kalkis_yayinci = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.inis_yayinci = self.create_publisher(Empty, '/tello/land', 10)

        # ROS subscriber'ları
        self.create_subscription(Image, '/tello/image_raw', self.goruntu_callback, 10)
        self.create_subscription(BatteryState, '/tello/battery', self.pil_callback, 10)
        self.create_subscription(String, '/tello/state', self.durum_callback, 10)

        # Ek durum değişkenleri
        self.mevcut_goruntu = None
        self.pil_seviyesi = 0
        self.drone_durumu = ""

        self.get_logger().info("Tello Nesne Tespit node'u başlatıldı")

    def pil_callback(self, veri: BatteryState) -> None:
        """
        Pil bilgilerini işler.

        Args:
            veri (BatteryState): Drone'dan gelen pil durumu mesajı.
        """
        self.pil_seviyesi = veri.percentage
        self.get_logger().info(f"Batarya seviyesi: %{self.pil_seviyesi}")

    def durum_callback(self, veri: String) -> None:
        """
        Drone'un durum bilgisini işler.

        Args:
            veri (String): Uçuş durumu bilgisi.
        """
        self.drone_durumu = veri.data
        self.get_logger().info(f"Drone durumu: {self.drone_durumu}")

    def hiz_komutu_gonder(self, x: float, y: float, z: float, yaw: float) -> None:
        """
        Drone'a hız komutu gönderir.

        Args:
            x (float): İleri/geri hareket (pozitif: ileri, negatif: geri)
            y (float): Sola/sağa hareket (pozitif: sağa, negatif: sola)
            z (float): Yukarı/aşağı hareket
            yaw (float): Yaw dönüş komutu
        """
        twist_mesaji = Twist()
        twist_mesaji.linear.x = float(x)
        twist_mesaji.linear.y = float(y)
        twist_mesaji.linear.z = float(z)
        twist_mesaji.angular.z = float(yaw)
        self.hiz_komutu_yayinci.publish(twist_mesaji)

    def kalkis(self) -> None:
        """
        Drone'u kalkışa geçirir.
        """
        self.get_logger().info("Drone kalkıyor...")
        self.kalkis_yayinci.publish(Empty())
        time.sleep(5)  # Kalkış tamamlanana kadar bekle

    def inis(self) -> None:
        """
        Drone'u inişe geçirir.
        """
        self.get_logger().info("Drone iniş yapıyor...")
        self.inis_yayinci.publish(Empty())

    def nesne_ara(self) -> None:
        """
        Drone'a dönerek nesne arama modunu başlatır.
        Nesne bulunursa arama durur. 10 saniye sonunda hala nesne yoksa iniş yapılır.
        """
        if not self.arama_yapiliyor:
            self.get_logger().info("Arama moduna geçiliyor")
            self.kalkis()
            self.arama_yapiliyor = True
            self.arama_baslangic_zamani = self.get_clock().now().nanoseconds / 1e9

        mevcut_zaman = self.get_clock().now().nanoseconds / 1e9
        if mevcut_zaman - self.arama_baslangic_zamani < 10.0:
            self.hiz_komutu_gonder(0, 0, 0, 0.2)  # Yavaşça dön
        else:
            self.get_logger().info("Arama tamamlandı")
            self.arama_yapiliyor = False
            if not self.nesne_bulundu:
                self.get_logger().info("Nesne bulunamadı, drone iniş yapıyor")
                self.inis()
            self.hiz_komutu_gonder(0, 0, 0, 0)  # Dönmeyi durdur

    def kareyi_isle(self, kare: np.ndarray) -> np.ndarray:
        """
        Kameradan alınan karede nesne tespiti yapar ve drone'u yönlendirir.

        Args:
            kare (np.ndarray): İşlenecek görüntü karesi (BGR formatında).

        Returns:
            np.ndarray: Görselleştirme amacıyla nesne çerçevesi çizilmiş kare.
        """
        if kare is None:
            return None

        sonuclar = self.model(kare)
        tespitler = sonuclar[0].boxes.data
        islenmisCerceveKare = kare.copy()

        if len(tespitler) > 0:
            self.nesne_bulundu = True
            if self.arama_yapiliyor:
                self.get_logger().info("Nesne bulundu, arama durduruluyor")
                self.arama_yapiliyor = False
                self.hiz_komutu_gonder(0, 0, 0, 0)

            tespit = tespitler[0]
            x1, y1, x2, y2, guven, sinif = tespit.cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Çizim işlemleri
            cv2.rectangle(islenmisCerceveKare, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(islenmisCerceveKare, f"Class: {int(sinif)} Conf: {guven:.2f}",
                        (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Konuma göre hareket
            if cx < self.cercevGenislik // 2 - self.oludBolge:
                self.get_logger().info("Sola hareket")
                self.hiz_komutu_gonder(0, -0.2, 0, 0)
            elif cx > self.cercevGenislik // 2 + self.oludBolge:
                self.get_logger().info("Sağa hareket")
                self.hiz_komutu_gonder(0, 0.2, 0, 0)
            elif cy < self.cercevYukseklik // 2 - self.oludBolge:
                self.get_logger().info("İleri hareket")
                self.hiz_komutu_gonder(0.2, 0, 0, 0)
            elif cy > self.cercevYukseklik // 2 + self.oludBolge:
                self.get_logger().info("Geri hareket")
                self.hiz_komutu_gonder(-0.2, 0, 0, 0)
            else:
                self.get_logger().info("Drone merkezde")
                self.hiz_komutu_gonder(0, 0, 0, 0)
        else:
            if not self.arama_yapiliyor and self.nesne_bulundu:
                self.get_logger().info("Nesne kaybedildi, yeniden aranıyor")
                self.nesne_bulundu = False
                self.nesne_ara()

        return islenmisCerceveKare

    def goruntu_callback(self, veri: Image) -> None:
        """
        Kameradan gelen ROS görüntüsünü işler ve nesne takibini başlatır.

        Args:
            veri (Image): ROS formatında kamera verisi.

        Raises:
            Exception: OpenCV dönüşümlerinde hata olursa loglar.
        """
        try:
            cv_goruntu = self.kopru.imgmsg_to_cv2(veri, "bgr8")
            self.mevcut_goruntu = cv2.resize(cv_goruntu, (self.cercevGenislik, self.cercevYukseklik))

            if self.arama_yapiliyor:
                sonuclar = self.model(self.mevcut_goruntu)
                if len(sonuclar[0].boxes.data) > 0:
                    self.nesne_bulundu = True
                    self.get_logger().info("Arama sırasında nesne bulundu")
            elif self.nesne_bulundu:
                islenmis_kare = self.kareyi_isle(self.mevcut_goruntu)
                if islenmis_kare is not None:
                    cv2.imshow("Drone Takip", islenmis_kare)
                    cv2.waitKey(1)
            else:
                self.nesne_ara()

        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {str(e)}")

    def calistir(self) -> None:
        """
        Başlangıç fonksiyonudur. Pil seviyesi loglanır ve nesne arama süreci başlatılır.
        """
        self.get_logger().info(f"Başlangıç batarya seviyesi: %{self.pil_seviyesi}")
        self.nesne_ara()

def main(args=None):
    """Ana ROS2 giriş fonksiyonu"""
    rclpy.init(args=args)
    tespit_edici = TelloNesneTespit()

    try:
        tespit_edici.calistir()
        rclpy.spin(tespit_edici)
    except KeyboardInterrupt:
        tespit_edici.get_logger().info("Kapatılıyor...")
        tespit_edici.inis()
        cv2.destroyAllWindows()
    finally:
        tespit_edici.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()