import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LIDAR verisinin geldiği varsayılan ROS2 topic adı
            self.lidar_callback,
            10  # QoS profili
        )
        self.subscription  # Önleme amacıyla referans tutuyoruz
    
    def lidar_callback(self, msg):
        self.get_logger().info(f'LIDAR Verisi Alındı: {len(msg.ranges)} nokta')
        self.get_logger().info(f'Ön: {msg.ranges[len(msg.ranges)//2]} metre')  # Ortadaki veri genellikle ön noktaya karşılık gelir
        self.get_logger().info(f'__________________________________')  # Ortadaki veri genellikle ön noktaya karşılık gelir

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
