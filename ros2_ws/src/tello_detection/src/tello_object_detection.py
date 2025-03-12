#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import time

class TelloNesneTespit:
    def __init__(self):
        # ROS düğümünü başlat
        rospy.init_node('tello_nesne_tespit', anonymous=True)
        
        # Parametreler
        self.cercevGenislik = 640
        self.cercevYukseklik = 480
        self.oludBolge = 50  # Merkezdeki tolerans bölgesi
        self.nesne_bulundu = False
        self.arama_yapiliyor = False
        self.arama_baslangic_zamani = 0
        
        # YOLO modeli
        self.model = YOLO("best3.pt")  # Model dosyanızın yolu
        
        # CV Bridge
        self.kopru = CvBridge()
        
        # Yayıncılar (Publishers)
        self.hiz_komutu_yayinci = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.kalkis_yayinci = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.inis_yayinci = rospy.Publisher('/tello/land', Empty, queue_size=10)
        
        # Abonelikler (Subscribers)
        rospy.Subscriber('/tello/image_raw', Image, self.goruntu_callback)
        rospy.Subscriber('/tello/battery', BatteryState, self.pil_callback)
        rospy.Subscriber('/tello/state', String, self.durum_callback)
        
        # Değişkenler
        self.mevcut_goruntu = None
        self.pil_seviyesi = 0
        self.drone_durumu = ""
        
        rospy.loginfo("Tello Nesne Tespit düğümü başlatıldı")

    def pil_callback(self, veri):
        """Pil bilgisini işle"""
        self.pil_seviyesi = veri.percentage
        rospy.loginfo(f"Pil seviyesi: %{self.pil_seviyesi}")

    def durum_callback(self, veri):
        """Drone durum bilgisini işle"""
        self.drone_durumu = veri.data
        # rospy.loginfo(f"Drone durumu: {self.drone_durumu}")

    def hiz_komutu_gonder(self, x, y, z, yaw):
        """Drone'a hız komutları gönder"""
        twist_mesaji = Twist()
        twist_mesaji.linear.x = x  # ileri/geri
        twist_mesaji.linear.y = y  # sol/sağ
        twist_mesaji.linear.z = z  # yukarı/aşağı
        twist_mesaji.angular.z = yaw  # yaw rotasyonu
        self.hiz_komutu_yayinci.publish(twist_mesaji)

    def kalkis(self):
        """Drone'a kalkış komutu ver"""
        rospy.loginfo("Drone kalkış yapıyor...")
        self.kalkis_yayinci.publish(Empty())
        time.sleep(5)  # Kalkışın tamamlanmasını bekle

    def inis(self):
        """Drone'a iniş komutu ver"""
        rospy.loginfo("Drone iniş yapıyor...")
        self.inis_yayinci.publish(Empty())

    def nesne_ara(self):
        """360 derece dönerek nesne ara"""
        if not self.arama_yapiliyor:
            rospy.loginfo("Arama modu başlatılıyor")
            self.kalkis()
            self.arama_yapiliyor = True
            self.arama_baslangic_zamani = rospy.get_time()
            
        # Yaklaşık 10 saniye boyunca dönmeye devam et
        mevcut_zaman = rospy.get_time()
        if mevcut_zaman - self.arama_baslangic_zamani < 10.0:
            self.hiz_komutu_gonder(0, 0, 0, 0.2)  # Yavaşça dön
        else:
            rospy.loginfo("Arama tamamlandı")
            self.arama_yapiliyor = False
            if not self.nesne_bulundu:
                rospy.loginfo("Arama sırasında nesne bulunamadı, iniş yapılıyor")
                self.inis()
            self.hiz_komutu_gonder(0, 0, 0, 0)  # Dönmeyi durdur

    def kareyi_isle(self, kare):
        """Kareyi işleyerek nesneleri tespit et ve drone'u kontrol et"""
        if kare is None:
            return None
            
        sonuclar = self.model(kare)  # YOLO modeli ile tespit
        tespitler = sonuclar[0].boxes.data  # Tespit edilen nesneler
        
        islenmisCerceveKare = kare.copy()
        
        if len(tespitler) > 0:
            self.nesne_bulundu = True
            if self.arama_yapiliyor:
                rospy.loginfo("Nesne bulundu, arama durduruluyor")
                self.arama_yapiliyor = False
                self.hiz_komutu_gonder(0, 0, 0, 0)  # Dönmeyi durdur
                
            # İlk tespit edilen nesneyi işle
            tespit = tespitler[0]
            x1, y1, x2, y2, guven, sinif = tespit.cpu().numpy()
            cx = int((x1 + x2) / 2)  # Nesnenin merkez X koordinatı
            cy = int((y1 + y2) / 2)  # Nesnenin merkez Y koordinatı
            
            # Sınırlayıcı kutu ve bilgileri çiz
            cv2.rectangle(islenmisCerceveKare, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(islenmisCerceveKare, f"Sınıf: {int(sinif)} Güven: {guven:.2f}", 
                        (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Nesne konumuna göre drone hareketini kontrol et
            if cx < self.cercevGenislik // 2 - self.oludBolge:
                rospy.loginfo("Drone sola hareket ediyor")
                self.hiz_komutu_gonder(0, -0.2, 0, 0)  # Sola hareket
            elif cx > self.cercevGenislik // 2 + self.oludBolge:
                rospy.loginfo("Drone sağa hareket ediyor")
                self.hiz_komutu_gonder(0, 0.2, 0, 0)  # Sağa hareket
            elif cy < self.cercevYukseklik // 2 - self.oludBolge:
                rospy.loginfo("Drone ileri hareket ediyor")
                self.hiz_komutu_gonder(0.2, 0, 0, 0)  # İleri hareket
            elif cy > self.cercevYukseklik // 2 + self.oludBolge:
                rospy.loginfo("Drone geri hareket ediyor")
                self.hiz_komutu_gonder(-0.2, 0, 0, 0)  # Geri hareket
            else:
                rospy.loginfo("Drone sabit pozisyonda")
                self.hiz_komutu_gonder(0, 0, 0, 0)  # Sabit kal
        else:
            if not self.arama_yapiliyor and self.nesne_bulundu:
                rospy.loginfo("Nesne kaybedildi, tekrar aranıyor")
                self.nesne_bulundu = False
                self.nesne_ara()
        
        return islenmisCerceveKare

    def goruntu_callback(self, veri):
        """Drone'dan gelen görüntüyü işle"""
        try:
            cv_goruntu = self.kopru.imgmsg_to_cv2(veri, "bgr8")
            self.mevcut_goruntu = cv2.resize(cv_goruntu, (self.cercevGenislik, self.cercevYukseklik))
            
            if self.arama_yapiliyor:
                # Arama aşamasında sadece nesneleri tespit et, kontrol etme
                sonuclar = self.model(self.mevcut_goruntu)
                if len(sonuclar[0].boxes.data) > 0:
                    self.nesne_bulundu = True
                    rospy.loginfo("Arama sırasında nesne bulundu")
            elif self.nesne_bulundu:
                # Nesneyi normal şekilde takip et
                islenmis_kare = self.kareyi_isle(self.mevcut_goruntu)
                if islenmis_kare is not None:
                    cv2.imshow("Drone Takip", islenmis_kare)
                    cv2.waitKey(1)
            else:
                # Arama yapmıyorsa ve nesne bulunmadıysa, aramaya başla
                self.nesne_ara()
                
        except Exception as e:
            rospy.logerr(f"Görüntü işleme hatası: {e}")

    def calistir(self):
        """Ana yürütme fonksiyonu"""
        rospy.loginfo(f"Başlangıç pil seviyesi: %{self.pil_seviyesi}")
        
        # Nesne arama işlemini başlat
        self.nesne_ara()
        
        # Düğümü çalışır durumda tut
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Kapatılıyor")
            self.inis()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tespit_edici = TelloNesneTespit()
        tespit_edici.calistir()
    except rospy.ROSInterruptException:
        pass