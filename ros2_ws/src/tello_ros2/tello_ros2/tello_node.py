#!/usr/bin/env python3
# tello_node.py - ROS 2 Humble için Tello Drone düğümü

import socket
import threading
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, BatteryState
from cv_bridge import CvBridge

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')
        # Tello bağlantı parametreleri
        self.tello_ip = '192.168.10.1'  # Tello'nun IP adresi
        self.tello_port = 8889  # Tello komut portu
        self.tello_video_port = 11111  # Tello video portu
        self.local_ip = '0.0.0.0'
        self.local_port = 8890  # Yerel komut portu
        
        # Socket'ler
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind((self.local_ip, self.local_port))
        
        # ROS 2 yayıncıları (publishers)
        self.state_pub = self.create_publisher(String, 'tello/state', 10)
        self.image_pub = self.create_publisher(Image, 'tello/image_raw', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'tello/battery', 10)
        
        # ROS 2 abonelikleri (subscriptions)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'tello/cmd_vel', self.cmd_vel_callback, 10)
        self.takeoff_sub = self.create_subscription(
            Empty, 'tello/takeoff', self.takeoff_callback, 10)
        self.land_sub = self.create_subscription(
            Empty, 'tello/land', self.land_callback, 10)
        self.flip_sub = self.create_subscription(
            String, 'tello/flip', self.flip_callback, 10)
        
        # Drone ile iletişim için thread'ler
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.state_thread = threading.Thread(target=self.receive_state)
        self.video_thread = threading.Thread(target=self.receive_video)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Drone'u başlatma
        self.get_logger().info("Tello Node başlatılıyor...")
        self.send_command("command")
        time.sleep(1)
        self.send_command("streamon")
        time.sleep(1)
        
        # Thread'leri başlatma
        self.receive_thread.daemon = True
        self.state_thread.daemon = True
        self.video_thread.daemon = True
        self.receive_thread.start()
        self.state_thread.start()
        self.video_thread.start()
        
        self.get_logger().info("Tello Node hazır")
        
    def send_command(self, command):
        """Tello'ya komut gönderir"""
        self.get_logger().info(f"Komut gönderiliyor: {command}")
        self.command_socket.sendto(command.encode('utf-8'), (self.tello_ip, self.tello_port))
    
    def receive_response(self):
        """Tello'dan gelen yanıtları alır"""
        while True:
            try:
                response, _ = self.command_socket.recvfrom(1024)
                response = response.decode('utf-8')
                self.get_logger().info(f"Yanıt alındı: {response}")
            except Exception as e:
                self.get_logger().error(f"Yanıt alınamadı: {e}")
                break
    
    def receive_state(self):
        """Tello durumunu alır ve yayınlar"""
        state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        state_socket.bind((self.local_ip, 8890))
        
        while True:
            try:
                state, _ = state_socket.recvfrom(1024)
                state = state.decode('utf-8')
                
                # Durum bilgisini yayınla
                msg = String()
                msg.data = state
                self.state_pub.publish(msg)
                
                # Pil durumunu çıkar ve yayınla
                state_dict = dict([s.split(':') for s in state.split(';') if ':' in s])
                if 'bat' in state_dict:
                    battery_msg = BatteryState()
                    battery_msg.percentage = float(state_dict['bat']) / 100.0
                    battery_msg.voltage = 3.8  # Tahmini değer
                    self.battery_pub.publish(battery_msg)
                
            except Exception as e:
                self.get_logger().error(f"Durum alınamadı: {e}")
                break
    
    def receive_video(self):
        """Video akışını alır ve ROS topicine yayınlar"""
        cap = cv2.VideoCapture(f'udp://@0.0.0.0:11111')
        
        if not cap.isOpened():
            self.get_logger().error("Video akışı alınamadı!")
            return
        self.get_logger().warn("Veri akışoı aktif")

        while True:
            ret, frame = cap.read()
            if ret:
                # Görüntüyü ROS mesajına dönüştür ve yayınla
                self.get_logger().warn("Kare okundu")
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().error(f"Görüntü dönüştürülemedi: {e}")
            else:
                self.get_logger().warn("Kare okunamadı")
                time.sleep(0.1)
    
    def cmd_vel_callback(self, msg):
        """Hız komutlarını işler"""
        # Tello hız değerlerini -100 ile 100 arasında bekler
        x = int(msg.linear.x * 100)  # ileri/geri
        y = int(msg.linear.y * 100)  # sağ/sol
        z = int(msg.linear.z * 100)  # yukarı/aşağı
        yaw = int(msg.angular.z * 100)  # dönüş
        
        # Değerleri -100 ile 100 arasında sınırla
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        z = max(-100, min(100, z))
        yaw = max(-100, min(100, yaw))
        
        # Tello'ya rc komutu gönder
        self.send_command(f'rc {x} {y} {z} {yaw}')
    
    def takeoff_callback(self, msg):
        """Kalkış komutu"""
        self.send_command('takeoff')
    
    def land_callback(self, msg):
        """İniş komutu"""
        self.send_command('land')
    
    def flip_callback(self, msg):
        """Takla atma komutu (f:l, f:r, f:f, f:b)"""
        direction = msg.data.lower()
        if direction in ['l', 'r', 'f', 'b']:
            self.send_command(f'flip {direction}')
        else:
            self.get_logger().warn(f"Geçersiz takla yönü: {direction}")

def main(args=None):
    rclpy.init(args=args)
    tello_node = TelloNode()
    
    try:
        rclpy.spin(tello_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Temizlik ve kapatma
        tello_node.send_command('streamoff')
        tello_node.send_command('land')
        tello_node.command_socket.close()
        tello_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()