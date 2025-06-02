#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from std_msgs.msg import Empty, String


class TelloNesneAnaliz(Node):
    
    def __init__(self):
        super.__init__('tello_nesne_analiz')



        self.create_subscription(Image, '/tello/image_raw', self.goruntu_callback, 10)
        self.create_subscription(BatteryState,'/tello/battery',self.pil_callback, 10)
        self.create_subscription(String, '/tello/state',self.durum_callback, 10)

    def durum_callback(self, veri:String):
        self.drone_durumu = veri.data
        self.get_logger().info(f"Drone durumu: {self.drone_durumu}")

    def pil_callback(self, veri: BatteryState):
        self.pil_seviyesi = veri.percentage
        self.get_logger().info(f"Batarya seviyesi %{self.pil_seviyesi}")
        return

    def goruntu_callback(self, veri: Image):
        self.get_logger().info("Kamera görüntüsü geldi")
        try:
            cv_goruntu = CvBridge().imgmsg_to_cv2(veri, "bgr8")
            self.mevcut_goruntu = cv
            pass
        except Exception as e:
            raise e
        return
    

