import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter([], [], s=2)
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.ion()
        plt.show()

    def lidar_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x = np.array(msg.ranges) * np.cos(angles)
        y = np.array(msg.ranges) * np.sin(angles)

        self.sc.set_offsets(np.c_[x, y])
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
