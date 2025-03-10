import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math

class LidarWorldMapper(Node):
    def __init__(self):
        super().__init__('lidar_world_mapper')
        
        # LaserScan verisi için abonelik
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        # Robot pozisyonu için abonelik (opsiyonel - TF yerine doğrudan pozisyon kullanmak isterseniz)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose',  # Robotunuzun pozisyon topic'i
            self.pose_callback,
            10)
            
        # TF2 Buffer ve Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot pozisyonu
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Matplotlib kurulumu
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.lidar_points = self.ax.scatter([], [], s=2, c='red', label='LiDAR')
        self.robot_position = self.ax.scatter([], [], s=100, c='blue', marker='>', label='Robot')
        self.robot_path = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.5)[0]
        
        # Robot yolu için veri
        self.path_x = []
        self.path_y = []
        
        # Harita ayarları
        plt.xlim(-20, 20)  # Genişletilmiş görüş alanı
        plt.ylim(-20, 20)
        plt.grid(True)
        plt.title('Dünya Koordinatlarında LiDAR Haritalama')
        plt.xlabel('X (metre)')
        plt.ylabel('Y (metre)')
        plt.legend()
        plt.ion()
        plt.show()
        
        self.get_logger().info('LiDAR Dünya Haritalayıcı başlatıldı')

    def pose_callback(self, msg):
        # Robot pozisyonunu doğrudan güncelleyin (TF kullanmıyorsanız)
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        
        # Quaternion'dan Euler açısı (yaw) hesaplama
        q = msg.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                                    
        # Robot yolunu güncelle
        self.path_x.append(self.robot_x)
        self.path_y.append(self.robot_y)

    def get_transform(self, source_frame, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            return transform
        except TransformException as ex:
            self.get_logger().warning(f'TF alınamadı: {ex}')
            return None

    def lidar_callback(self, msg):
        # TF kullanarak robot pozisyonunu al (pose_callback kullanmıyorsanız)
        transform = self.get_transform('base_link', 'map')  # Robot'tan haritaya dönüşüm
        if transform:
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # Robot yolunu güncelle
            if not self.path_x or (self.robot_x != self.path_x[-1] or self.robot_y != self.path_y[-1]):
                self.path_x.append(self.robot_x)
                self.path_y.append(self.robot_y)
        
        # LiDAR verilerini işle
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        # Geçersiz değerleri filtrele (Inf veya NaN)
        valid_indices = np.isfinite(msg.ranges)
        valid_ranges = np.array(msg.ranges)[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) > 0:
            # Robot koordinat sisteminde noktaları hesapla
            local_x = valid_ranges * np.cos(valid_angles)
            local_y = valid_ranges * np.sin(valid_angles)
            
            # Dünya koordinat sistemine dönüştür
            world_x = local_x * np.cos(self.robot_yaw) - local_y * np.sin(self.robot_yaw) + self.robot_x
            world_y = local_x * np.sin(self.robot_yaw) + local_y * np.cos(self.robot_yaw) + self.robot_y
            
            # Görselleştirmeyi güncelle
            self.lidar_points.set_offsets(np.c_[world_x, world_y])
            self.robot_position.set_offsets(np.c_[self.robot_x, self.robot_y])
            
            # Robot yönünü göster
            dir_x = 0.5 * np.cos(self.robot_yaw) + self.robot_x
            dir_y = 0.5 * np.sin(self.robot_yaw) + self.robot_y
            
            # Robot yolunu güncelle
            self.robot_path.set_data(self.path_x, self.path_y)
            
            # Harita görünümünü otomatik ayarla (opsiyonel)
            if len(world_x) > 0:
                all_x = np.concatenate([world_x, [self.robot_x]])
                all_y = np.concatenate([world_y, [self.robot_y]])
                x_min, x_max = np.min(all_x) - 2, np.max(all_x) + 2
                y_min, y_max = np.min(all_y) - 2, np.max(all_y) + 2
                
                # Minimum harita boyutu
                x_span = max(5, x_max - x_min)
                y_span = max(5, y_max - y_min)
                
                plt.xlim(x_min, x_max)
                plt.ylim(y_min, y_max)
            
            plt.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LidarWorldMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Kapat
        plt.ioff()
        plt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()