#!/usr/bin/env python3
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
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('tello_nesne_tespit')
        
        # Parameters
        self.cercevGenislik = 640
        self.cercevYukseklik = 480
        self.oludBolge = 50  # Dead zone tolerance in the center
        self.nesne_bulundu = False
        self.arama_yapiliyor = False
        self.arama_baslangic_zamani = 0
        
        # YOLO model
        self.model = YOLO("best3.pt")  # Path to your model file
        
        # CV Bridge
        self.kopru = CvBridge()
        
        # Publishers
        self.hiz_komutu_yayinci = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.kalkis_yayinci = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.inis_yayinci = self.create_publisher(Empty, '/tello/land', 10)
        
        # Subscribers
        self.create_subscription(Image, '/tello/image_raw', self.goruntu_callback, 10)
        self.create_subscription(BatteryState, '/tello/battery', self.pil_callback, 10)
        self.create_subscription(String, '/tello/state', self.durum_callback, 10)
        
        # Variables
        self.mevcut_goruntu = None
        self.pil_seviyesi = 0
        self.drone_durumu = ""
        
        self.get_logger().info("Tello Nesne Tespit node started")

    def pil_callback(self, veri):
        """Process battery information"""
        self.pil_seviyesi = veri.percentage
        self.get_logger().info(f"Battery level: %{self.pil_seviyesi}")

    def durum_callback(self, veri):
        """Process drone state information"""
        self.drone_durumu = veri.data
        # self.get_logger().info(f"Drone state: {self.drone_durumu}")

    def hiz_komutu_gonder(self, x, y, z, yaw):
        """Send velocity commands to the drone"""
        twist_mesaji = Twist()
        twist_mesaji.linear.x = float(x)  # forward/backward
        twist_mesaji.linear.y = float(y)  # left/right
        twist_mesaji.linear.z = float(z)  # up/down
        twist_mesaji.angular.z = float(yaw)  # yaw rotation
        self.hiz_komutu_yayinci.publish(twist_mesaji)

    def kalkis(self):
        """Command the drone to take off"""
        self.get_logger().info("Drone is taking off...")
        self.kalkis_yayinci.publish(Empty())
        time.sleep(5)  # Wait for takeoff to complete

    def inis(self):
        """Command the drone to land"""
        self.get_logger().info("Drone is landing...")
        self.inis_yayinci.publish(Empty())

    def nesne_ara(self):
        """Search for objects by rotating 360 degrees"""
        if not self.arama_yapiliyor:
            self.get_logger().info("Starting search mode")
            self.kalkis()
            self.arama_yapiliyor = True
            self.arama_baslangic_zamani = self.get_clock().now().nanoseconds / 1e9
            
        # Keep rotating for about 10 seconds
        mevcut_zaman = self.get_clock().now().nanoseconds / 1e9
        if mevcut_zaman - self.arama_baslangic_zamani < 10.0:
            self.hiz_komutu_gonder(0, 0, 0, 0.2)  # Rotate slowly
        else:
            self.get_logger().info("Search completed")
            self.arama_yapiliyor = False
            if not self.nesne_bulundu:
                self.get_logger().info("No object found during search, landing")
                self.inis()
            self.hiz_komutu_gonder(0, 0, 0, 0)  # Stop rotation

    def kareyi_isle(self, kare):
        """Process the frame to detect objects and control the drone"""
        if kare is None:
            return None
            
        sonuclar = self.model(kare)  # Detection with YOLO model
        tespitler = sonuclar[0].boxes.data  # Detected objects
        
        islenmisCerceveKare = kare.copy()
        
        if len(tespitler) > 0:
            self.nesne_bulundu = True
            if self.arama_yapiliyor:
                self.get_logger().info("Object found, stopping search")
                self.arama_yapiliyor = False
                self.hiz_komutu_gonder(0, 0, 0, 0)  # Stop rotation
                
            # Process the first detected object
            tespit = tespitler[0]
            x1, y1, x2, y2, guven, sinif = tespit.cpu().numpy()
            cx = int((x1 + x2) / 2)  # Object center X coordinate
            cy = int((y1 + y2) / 2)  # Object center Y coordinate
            
            # Draw bounding box and info
            cv2.rectangle(islenmisCerceveKare, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(islenmisCerceveKare, f"Class: {int(sinif)} Conf: {guven:.2f}", 
                        (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Control drone movement based on object position
            if cx < self.cercevGenislik // 2 - self.oludBolge:
                self.get_logger().info("Drone moving left")
                self.hiz_komutu_gonder(0, -0.2, 0, 0)  # Move left
            elif cx > self.cercevGenislik // 2 + self.oludBolge:
                self.get_logger().info("Drone moving right")
                self.hiz_komutu_gonder(0, 0.2, 0, 0)  # Move right
            elif cy < self.cercevYukseklik // 2 - self.oludBolge:
                self.get_logger().info("Drone moving forward")
                self.hiz_komutu_gonder(0.2, 0, 0, 0)  # Move forward
            elif cy > self.cercevYukseklik // 2 + self.oludBolge:
                self.get_logger().info("Drone moving backward")
                self.hiz_komutu_gonder(-0.2, 0, 0, 0)  # Move backward
            else:
                self.get_logger().info("Drone in stable position")
                self.hiz_komutu_gonder(0, 0, 0, 0)  # Stay still
        else:
            if not self.arama_yapiliyor and self.nesne_bulundu:
                self.get_logger().info("Object lost, searching again")
                self.nesne_bulundu = False
                self.nesne_ara()
        
        return islenmisCerceveKare

    def goruntu_callback(self, veri):
        """Process images from the drone"""
        try:
            cv_goruntu = self.kopru.imgmsg_to_cv2(veri, "bgr8")
            self.mevcut_goruntu = cv2.resize(cv_goruntu, (self.cercevGenislik, self.cercevYukseklik))
            
            if self.arama_yapiliyor:
                # In search phase, only detect objects, don't control yet
                sonuclar = self.model(self.mevcut_goruntu)
                if len(sonuclar[0].boxes.data) > 0:
                    self.nesne_bulundu = True
                    self.get_logger().info("Object found during search")
            elif self.nesne_bulundu:
                # Track the object normally
                islenmis_kare = self.kareyi_isle(self.mevcut_goruntu)
                if islenmis_kare is not None:
                    cv2.imshow("Drone Tracking", islenmis_kare)
                    cv2.waitKey(1)
            else:
                # If not searching and no object found, start search
                self.nesne_ara()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def calistir(self):
        """Main execution function"""
        self.get_logger().info(f"Initial battery level: %{self.pil_seviyesi}")
        
        # Start object search
        self.nesne_ara()

def main(args=None):
    rclpy.init(args=args)
    
    tespit_edici = TelloNesneTespit()
    
    try:
        rclpy.spin(tespit_edici)
    except KeyboardInterrupt:
        tespit_edici.get_logger().info("Shutting down")
        tespit_edici.inis()
        cv2.destroyAllWindows()
    finally:
        # Clean shutdown
        tespit_edici.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()