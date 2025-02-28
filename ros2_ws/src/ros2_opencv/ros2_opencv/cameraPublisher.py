import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')

        self.cameraDeviceNumber = 2
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        if not self.camera.isOpened():
            self.get_logger().error("Kamera açılamadı! Lütfen kamera bağlantısını kontrol edin.")
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 10  # ROS 2 queue size int olmalı
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.01  # 10 Hz (100ms)
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()

        if not success:
            self.get_logger().error("Kamera görüntüsü alınamadı!")
            return

        try:
            frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info(f'Resim paylaşıldı: {self.i}')
            self.i += 1
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {e}")

    def __del__(self):
        if self.camera.isOpened():
            self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
