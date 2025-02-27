import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'

        # QoS Profili oluştur (BEST_EFFORT veya RELIABLE seçilebilir)
        #qos_profile = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Abonelik oluştur
        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction, 10
        )
        self.i = 0

    def listener_callbackFunction(self, ROS2ImageMessage):
        frame = self.bridgeObject.imgmsg_to_cv2(ROS2ImageMessage, desired_encoding='bgr8')
        
        cv2.imshow('Camera video', frame)
        cv2.waitKey(1)  # Daha stabil olması için 10 ms beklet

        self.get_logger().info(f'Resim alındı {self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    subscriberObject = SubscriberNodeClass()
    
    rclpy.spin(subscriberObject)
    
    subscriberObject.destroy_node()  # Hatalı yazımı düzelttik
    
    rclpy.shutdown()  # Hatalı yazımı düzelttik

if __name__ == '__main__':
    main()
