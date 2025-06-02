import rclpy
from rclpy.node import Node
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import cv2
import time

class TelloNesneTespit(Node):
    def __init__(self):
        super().__init__('tello_nesne_tespit')

        self.oludBolge = 100

        self.nesne_bulundu = False
        self.arama_yapiliyor = False
        self.arama_baslangic_zamani = 0

        self.model = YOLO("/home/encoder/Desktop/ros2-robot/ros2_ws/src/tello_takip/tello_takip/best3.pt")
        self.kopru = CvBridge()

        self.hiz_komutu_yayinci = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.kalkis_yayinci = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.inis_yayinci = self.create_publisher(Empty, '/tello/land', 10)

        self.create_subscription(Image, '/tello/image_raw', self.goruntu_callback, 10)
        self.create_subscription(BatteryState, '/tello/battery', self.pil_callback, 10)
        self.create_subscription(String, '/tello/state', self.durum_callback, 10)

        self.mevcut_goruntu = None
        self.pil_seviyesi = 0
        self.drone_durumu = ""

        self.get_logger().info("Tello Nesne Tespit node'u başlatıldı")

    def pil_callback(self, veri: BatteryState) -> None:
        self.pil_seviyesi = veri.percentage

    def durum_callback(self, veri: String) -> None:
        self.drone_durumu = veri.data

    def hiz_komutu_gonder(self, x: float, y: float, z: float, yaw: float) -> None:
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.z = yaw
        self.hiz_komutu_yayinci.publish(twist)

    def kalkis(self):
        self.get_logger().info("Drone kalkıyor...")
        self.kalkis_yayinci.publish(Empty())
        time.sleep(5)

    def inis(self):
        self.get_logger().info("Drone iniş yapıyor...")
        self.inis_yayinci.publish(Empty())

    def nesne_ara(self):
        if not self.arama_yapiliyor:
            self.kalkis()
            self.arama_yapiliyor = True
            self.arama_baslangic_zamani = self.get_clock().now().nanoseconds / 1e9

        mevcut_zaman = self.get_clock().now().nanoseconds / 1e9
        if mevcut_zaman - self.arama_baslangic_zamani < 10.0:
            self.hiz_komutu_gonder(0, 0, 0, 0.2)
        else:
            self.arama_yapiliyor = False
            if not self.nesne_bulundu:
                self.get_logger().info("Nesne bulunamadı, drone iniş yapıyor")
                self.inis()
            self.hiz_komutu_gonder(0, 0, 0, 0)

    def nesne_tespiti_ve_koordinat(self, kare: np.ndarray):
        sonuclar = self.model(kare)
        tespitler = sonuclar[0].boxes.data
        islenmis_kare = kare.copy()

        if len(tespitler) > 0:
            tespit = tespitler[0]
            return islenmis_kare, tuple(tespit.tolist())
        else:
            return islenmis_kare, None

    def hareket_komutu_gonder(self, dx: int, dy: int):
        if abs(dx) > self.oludBolge:
            self.hiz_komutu_gonder(0, 0.2 if dx > 0 else -0.2, 0, 0)
        elif abs(dy) > self.oludBolge:
            self.hiz_komutu_gonder(0.2 if dy < 0 else -0.2, 0, 0, 0)
        else:
            self.hiz_komutu_gonder(0, 0, 0, 0)

    def kareyi_isle(self, kare: np.ndarray) -> np.ndarray:
        if kare is None:
            return None

        cerceve_yukseklik, cerceve_genislik = kare.shape[:2]
        kare_kopyasi, tespit = self.nesne_tespiti_ve_koordinat(kare)

        if tespit:
            self.nesne_bulundu = True
            if self.arama_yapiliyor:
                self.arama_yapiliyor = False
                self.hiz_komutu_gonder(0, 0, 0, 0)

            x1, y1, x2, y2, guven, sinif = tespit
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.rectangle(kare_kopyasi, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(kare_kopyasi, f"Class: {int(sinif)} Conf: {guven:.2f}",
                        (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            dx = cx - cerceve_genislik // 2
            dy = cy - cerceve_yukseklik // 2
            self.hareket_komutu_gonder(dx, dy)
        else:
            if self.nesne_bulundu:
                self.nesne_bulundu = False
            if not self.arama_yapiliyor:
                self.nesne_ara()

        return kare_kopyasi

    def goruntu_callback(self, veri: Image) -> None:
        try:
            cv_goruntu = self.kopru.imgmsg_to_cv2(veri, "bgr8")
            self.mevcut_goruntu = cv_goruntu

            if self.arama_yapiliyor or self.nesne_bulundu:
                islenmis_kare = self.kareyi_isle(self.mevcut_goruntu)
                if islenmis_kare is not None:
                    cv2.imshow("Drone Takip", islenmis_kare)
                    cv2.waitKey(1)
            else:
                self.nesne_ara()
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {str(e)}")

    def calistir(self):
        self.get_logger().info(f"Başlangıç batarya seviyesi: %{self.pil_seviyesi}")
        self.nesne_ara()


def main(args=None):
    rclpy.init(args=args)
    tespit_node = TelloNesneTespit()
    try:
        tespit_node.calistir()
        rclpy.spin(tespit_node)
    except KeyboardInterrupt:
        tespit_node.inis()
        cv2.destroyAllWindows()
    finally:
        tespit_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
