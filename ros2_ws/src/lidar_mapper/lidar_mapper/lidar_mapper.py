import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
import math
import time
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class EnhancedLidarMapper(Node):
    def __init__(self):
        super().__init__('enhanced_lidar_mapper')
        
        # QoS profili oluştur
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # En iyi çaba ile iletim (veri kaybı olabilir ama hızlıdır)
            history=QoSHistoryPolicy.KEEP_LAST,  # Son birkaç mesajı saklar
            depth=10  # Hafızada en fazla 10 veri saklanır
        )
        
        # LaserScan verisi için abonelik
        self.laser_subscription = self.create_subscription(
            LaserScan,        # Mesaj türü: LaserScan (LiDAR'ın gönderdiği veri formatı)
            '/scan',          # LiDAR'ın veri yayınladığı ROS 2 topiği
            self.lidar_callback,  # Yeni veri geldiğinde çalışacak callback fonksiyonu
            qos_profile=lidar_qos  # Kalite Servisi Ayarları (QoS)
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,  # Mesaj türü: Odometry (robotun konum ve hız bilgileri)
            '/odom',   # Robotun anlık konum verisini yayınladığı topik
            self.odom_callback,  # Gelen veriyi işlemek için callback fonksiyonu
            10  # QoS parametresi: 10 mesajlık tampon bellek
        )

        # Engelleri MarkerArray formatında yayınlar.
        # MarkerArray, birden fazla nesne (örneğin engeller) içeren bir işaretleme formatıdır.
        # Her engel bir "marker" olarak işaretlenir ve /obstacle_markers topiğine yayınlanır.
        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray, 
            '/obstacle_markers', 
            10)
        
        
        # Robotun hareket ettiği yolu çizen bir marker yayınlar.
        # Marker.LINE_STRIP formatında çalışarak robotun geçtiği noktaları bağlayan bir çizgi çizer.
        # Bu marker /path_marker topiğine yayınlanır.
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/path_marker',
            10)
            
        # LiDAR verisine dayanarak oluşturulan haritayı yayınlar.
        # Harita, dolu/boş bölgeleri gösteren bir 2D grid olarak işlenir.
        # Marker.CUBE_LIST formatında çalışır ve /occupancy_grid topiğine yayın yapar.
        self.occupancy_map_pub = self.create_publisher(
            Marker,
            '/occupancy_grid',
            10)
        
        # TF2 Buffer ve Listener
        # ROS 2'nin TF2 (Transform) sistemini kullanarak robotun pozisyonunu takip eder.
        # tf_listener, farklı çerçeveler (örneğin, LiDAR verisi → harita çerçevesi) arasındaki dönüşümleri dinler.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot pozisyonu ve harita
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.path_x = []
        self.path_y = []
        
        # Occupancy grid map - basit 2D grid
        self.map_resolution = 0.05  # metre/hücre
        self.map_width = 1000       # hücre sayısı
        self.map_height = 1000
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2
        
        # Boş occupancy grid haritası (0: free, 100: occupied, -1: unknown)
        self.occupancy_grid = -1 * np.ones((self.map_height, self.map_width), dtype=np.int8)
        
        # Robot boyutları (metre)
        self.robot_width = 0.22
        self.robot_length = 0.22
        self.robot_radius = math.sqrt((self.robot_width/2)**2 + (self.robot_length/2)**2)
        
        # Harita güncelleme zamanı
        self.last_map_update_time = self.get_clock().now()
        self.map_update_period = 1.0  # saniye
        
        # Point cloud
        self.point_cloud = []  # (x, y, z, intensity, timestamp)
        
        # Matplotlib
        self.use_matplotlib = True
        if self.use_matplotlib:
            self.setup_visualization()
            
        self.get_logger().info('Geliştirilmiş LiDAR Haritalama düğümü başlatıldı')

    def setup_visualization(self):
        # Grafik penceresini (figure) ve eksenleri (axes) oluşturuyoruz.
        # Grafik 10x10 inç boyutunda olacak.
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
        # LiDAR verisi için boş bir scatter plot oluşturuyoruz.
        # Başlangıçta boş bir liste ile başlıyoruz, noktaların boyutu küçük olacak ve renkleri kırmızı olacak.
        # Etiket olarak "LiDAR" ekliyoruz.
        self.lidar_points = self.ax.scatter([], [], s=2, c='red', label='LiDAR')
        
        # Robotun konumunu göstermek için başka bir scatter plot oluşturuyoruz.
        # Robotun boyutları daha büyük (100) ve mavi renkte, işaretçi (marker) olarak '>' (sağa işaret eden ok) kullanıyoruz.
        # Etiket olarak "Robot" ekliyoruz.
        self.robot_position = self.ax.scatter([], [], s=100, c='blue', marker='>', label='Robot')
        
        # Robotun hareket ettiği yolu gösterecek bir çizgi oluşturuyoruz.
        # Çizgi başlangıçta boş olacak ve mavi renkte, şeffaflık %50 olacak.
        self.robot_path = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.5)[0]
        
        # Robotun gövdesini çizmek için bir dikdörtgen oluşturuyoruz.
        # Robotun boyutlarını (width ve length) kullanarak bu dikdörtgeni ekliyoruz.
        # Dikdörtgenin içi boş olacak ve kenar rengi mavi olacak.
        self.robot_footprint = plt.Rectangle((0, 0), self.robot_width, self.robot_length, 
                                            fill=False, edgecolor='blue', linewidth=2)
        # Dikdörtgeni (robotun gövdesini) eksene ekliyoruz.
        self.ax.add_patch(self.robot_footprint)
        
        # Grafik alanının X ve Y eksenlerinin sınırlarını belirliyoruz.
        # X ve Y eksenleri -5 ile +5 arasında olacak.
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        
        # Grafik üzerinde bir ızgara (grid) ekliyoruz, böylece veriler daha kolay okunabilir.
        plt.grid(True)
        
        # Grafik başlığını ve eksen etiketlerini belirliyoruz.
        plt.title('LiDAR Haritalama')  # Grafik başlığı
        plt.xlabel('X (metre)')        # X ekseni etiketi
        plt.ylabel('Y (metre)')        # Y ekseni etiketi

        # Grafik üzerinde etiketleri (legend) ekliyoruz.
        # LiDAR verileri ve robot için etiketler olacak.
        plt.legend()

        # Dinamik olarak güncellenebilmesi için 'interactive mode' aktif ediyoruz.
        plt.ion()

        # Grafik penceresini gösteriyoruz.
        plt.show()

    def odom_callback(self, msg):
        # Robot pozisyonunu güncelle
        self.robot_x = msg.pose.pose.position.x  # Odometri mesajından robotun X pozisyonunu alıyoruz.
        self.robot_y = msg.pose.pose.position.y  # Odometri mesajından robotun Y pozisyonunu alıyoruz.
        
        # Quaternion'dan Euler açısı (yaw) hesaplama
        q = msg.pose.pose.orientation  # Odometri mesajından robotun oryantasyonunu (yönünü) alıyoruz. Bu değer quaternion formatında gelir.
        # Quaternion'dan yaw açısını (dönme açısı) hesaplamak için atan2 fonksiyonu kullanıyoruz.
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))  # Bu hesaplama, quaternion'ı Euler açısına dönüştürür.
                                    
        # Robot yolunu güncelle
        # Yol, robotun X ve Y pozisyonlarını saklayan listelerdir. Bu kısmı sadece robotun pozisyonu değiştiğinde güncelliyoruz.
        if not self.path_x or (self.robot_x != self.path_x[-1] or self.robot_y != self.path_y[-1]):
            # Eğer yol listeleri boşsa ya da son pozisyon robotun mevcut pozisyonundan farklıysa, yeni pozisyonu ekliyoruz.
            self.path_x.append(self.robot_x)  # Robotun X pozisyonunu yol listesine ekliyoruz.
            self.path_y.append(self.robot_y)  # Robotun Y pozisyonunu yol listesine ekliyoruz.
            self.update_path_marker()  # Yol marker'ını güncelliyoruz, bu fonksiyon yolun görselleştirilmesini sağlar.

    def update_path_marker(self):
        # Robot yolunu gösteren marker (işaretçi) nesnesi oluşturuluyor
        path_marker = Marker()
        
        # Marker'ın başlık (header) bilgileri ayarlanıyor
        path_marker.header.frame_id = "map"  # Marker'ın hangi çerçeveye göre yerleştirileceğini belirliyoruz (harita çerçevesi)
        path_marker.header.stamp = self.get_clock().now().to_msg()  # Marker zaman damgası, şu anki zaman
        
        # Marker'ın diğer bilgileri ayarlanıyor
        path_marker.ns = "robot_path"  # Marker için bir isim uzayı (namespace) belirliyoruz
        path_marker.id = 0  # Marker'ın benzersiz kimliği (ID)
        path_marker.type = Marker.LINE_STRIP  # Marker tipi, bu durumda bir çizgi dizisi (line strip)
        path_marker.action = Marker.ADD  # Marker'ın eklenmesini istediğimizi belirtiyoruz
        path_marker.scale.x = 0.05  # Çizginin kalınlığını ayarlıyoruz (0.05 metre)

        # Marker'ın rengini belirliyoruz: mavi (RGB) ve saydamlık (a)
        path_marker.color.r = 0.0  # Kırmızı bileşeni
        path_marker.color.g = 0.0  # Yeşil bileşeni
        path_marker.color.b = 1.0  # Mavi bileşeni
        path_marker.color.a = 0.8  # Saydamlık (açıklık) değeri, 1.0 tamamen opak, 0.0 tamamen saydam

        # Yol noktalarını ekle
        for i in range(len(self.path_x)):  # Yolun her noktası için döngü
            p = Point()  # Yeni bir Point (nokta) oluşturuluyor
            p.x = self.path_x[i]  # Yolun X koordinatını alıyoruz
            p.y = self.path_y[i]  # Yolun Y koordinatını alıyoruz
            p.z = 0.01  # Z koordinatını (yükseklik) sabit tutuyoruz, zeminin biraz üstü (örneğin, robotun yüksekliği)
            path_marker.points.append(p)  # Bu noktayı marker'a ekliyoruz
        
        # Marker'ı yayınlıyoruz (publishing)
        self.path_marker_pub.publish(path_marker)

    def world_to_grid(self, x, y):
        # Dünya koordinatlarını ızgara indekslerine dönüştür
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Sınırları kontrol et
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            return grid_x, grid_y
        else:
            return None, None

    def update_occupancy_grid(self, world_x, world_y):
        # Occupancy grid haritasını güncelle
        for i in range(len(world_x)):
            x, y = world_x[i], world_y[i]
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if grid_x is not None and grid_y is not None:
                # Bu nokta engel
                self.occupancy_grid[grid_y, grid_x] = 100
                
                # Robot etrafındaki alanı (robotun boyutları kadar) boş olarak işaretle
                robot_grid_x, robot_grid_y = self.world_to_grid(self.robot_x, self.robot_y)
                if robot_grid_x is not None and robot_grid_y is not None:
                    # Robot konumunu ve 1 hücre etrafını boş olarak işaretle
                    robot_radius_grid = max(1, int(self.robot_radius / self.map_resolution))
                    
                    for ry in range(robot_grid_y - robot_radius_grid, robot_grid_y + robot_radius_grid + 1):
                        for rx in range(robot_grid_x - robot_radius_grid, robot_grid_x + robot_radius_grid + 1):
                            if 0 <= rx < self.map_width and 0 <= ry < self.map_height:
                                # Robot konumunda engel yok
                                if self.occupancy_grid[ry, rx] == -1:  # Sadece bilinmeyen hücreleri boş olarak işaretle
                                    self.occupancy_grid[ry, rx] = 0

    def publish_occupancy_grid(self):
        # Occupancy grid'i marker olarak yayınla (görselleştirme için)
        
        # Şu anki zaman ile son harita güncelleme zamanı arasındaki farkı kontrol et
        # Eğer periyodun altında bir fark varsa harita güncellenmez
        now = self.get_clock().now()
        if (now - self.last_map_update_time).nanoseconds / 1e9 < self.map_update_period:
            return  # Eğer periyot geçmediyse fonksiyon bitir

        # Zamanı güncelle
        self.last_map_update_time = now
        
        # Marker grid için yeni bir Marker objesi oluştur
        grid_marker = Marker()
        grid_marker.header.frame_id = "map"  # Harita çerçevesinde yer alacak
        grid_marker.header.stamp = now.to_msg()  # Zaman damgası ile güncelle
        grid_marker.ns = "occupancy_grid"  # Namespace, marker grubu için
        grid_marker.id = 0  # Marker ID'si
        grid_marker.type = Marker.CUBE_LIST  # Marker türü: Küp listesi
        grid_marker.action = Marker.ADD  # Marker ekleme aksiyonu
        
        # Izgara hücre boyutunu belirle
        grid_marker.scale.x = self.map_resolution  # X boyutu (hücre boyutu)
        grid_marker.scale.y = self.map_resolution  # Y boyutu (hücre boyutu)
        grid_marker.scale.z = 0.01  # Z boyutu (ince bir zemin katmanı)
        
        # Varsayılan renk (gri - bilinmeyen)
        grid_marker.color.r = 0.5  # Kırmızı (yarım gri)
        grid_marker.color.g = 0.5  # Yeşil (yarım gri)
        grid_marker.color.b = 0.5  # Mavi (yarım gri)
        grid_marker.color.a = 0.3  # Yarı şeffaflık
        
        # Dolu ızgara hücrelerini ekle
        for y in range(0, self.map_height, 4):  # Performansı artırmak için her 4 hücrede bir kontrol et
            for x in range(0, self.map_width, 4):
                if self.occupancy_grid[y, x] == 100:  # Eğer hücre engel (dolu) ise
                    p = Point()  # Yeni bir Point objesi oluştur
                    p.x = x * self.map_resolution + self.map_origin_x  # X koordinatını hesapla
                    p.y = y * self.map_resolution + self.map_origin_y  # Y koordinatını hesapla
                    p.z = 0.0  # Z koordinatı (zemin seviyesi)

                    color = ColorRGBA()  # Renk belirleme
                    color.r = 1.0  # Kırmızı - engel
                    color.g = 0.0  # Yeşil
                    color.b = 0.0  # Mavi
                    color.a = 0.8  # Opaklık (şeffaflık)

                    # Bu noktayı marker'a ekle
                    grid_marker.points.append(p)
                    grid_marker.colors.append(color)
                
                elif self.occupancy_grid[y, x] == 0:  # Eğer hücre boş ise
                    p = Point()  # Yeni bir Point objesi oluştur
                    p.x = x * self.map_resolution + self.map_origin_x  # X koordinatını hesapla
                    p.y = y * self.map_resolution + self.map_origin_y  # Y koordinatını hesapla
                    p.z = 0.0  # Z koordinatını sabit tut (zemin)

                    color = ColorRGBA()  # Renk belirleme
                    color.r = 0.0  # Kırmızı
                    color.g = 1.0  # Yeşil - boş alan
                    color.b = 0.0  # Mavi
                    color.a = 0.3  # Opaklık (şeffaflık)

                    # Bu noktayı marker'a ekle
                    grid_marker.points.append(p)
                    grid_marker.colors.append(color)
        
        # Hazırlanan occupancy grid marker'ını yayınla
        self.occupancy_map_pub.publish(grid_marker)

    def publish_obstacle_markers(self, world_x, world_y):
        # Engelleri göstermek için marker dizisi
        marker_array = MarkerArray()
        
        # Tüm engel noktalarını bir marker'a ekle
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "obstacle_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        
        # Nokta boyutu
        points_marker.scale.x = 0.05
        points_marker.scale.y = 0.05
        
        # Kırmızı renk
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 0.8
        
        # Engel noktalarını ekle
        for i in range(len(world_x)):
            p = Point()
            p.x = world_x[i]
            p.y = world_y[i]
            p.z = 0.05  # Zemin seviyesinin biraz üstünde
            points_marker.points.append(p)
            
        marker_array.markers.append(points_marker)
        
        # Robot dairesini gösteren marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 1
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        
        # Robot pozisyonu
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.05
        
        # Robot boyutu
        robot_marker.scale.x = self.robot_width
        robot_marker.scale.y = self.robot_length
        robot_marker.scale.z = 0.1
        
        # Mavi yarı-saydam renk
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        robot_marker.color.a = 0.5
        
        marker_array.markers.append(robot_marker)
        
        self.obstacle_marker_pub.publish(marker_array)

    def lidar_callback(self, msg):
        # LiDAR verilerini işle
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        # Geçersiz değerleri filtrele (Inf veya NaN)
        valid_indices = np.logical_and(np.isfinite(msg.ranges), np.array(msg.ranges) > 0.1)
        valid_ranges = np.array(msg.ranges)[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) > 0:
            # Robot koordinat sisteminde noktaları hesapla
            local_x = valid_ranges * np.cos(valid_angles)
            local_y = valid_ranges * np.sin(valid_angles)
            
            # Dünya koordinat sistemine dönüştür
            world_x = local_x * np.cos(self.robot_yaw) - local_y * np.sin(self.robot_yaw) + self.robot_x
            world_y = local_x * np.sin(self.robot_yaw) + local_y * np.cos(self.robot_yaw) + self.robot_y
            
            # Nokta bulutunu güncelle
            timestamp = self.get_clock().now().nanoseconds / 1e9
            for i in range(len(world_x)):
                self.point_cloud.append((world_x[i], world_y[i], 0.0, 1.0, timestamp))
            
            # Son 10000 noktayı tut (bellek yönetimi)
            if len(self.point_cloud) > 10000:
                self.point_cloud = self.point_cloud[-10000:]
            
            # Occupancy grid'i güncelle
            self.update_occupancy_grid(world_x, world_y)
            
            # RViz için marker'ları yayınla
            self.publish_obstacle_markers(world_x, world_y)
            self.publish_occupancy_grid()
            
            # Matplotlib ile görselleştir
            if self.use_matplotlib:
                self.update_visualization(world_x, world_y)

    def update_visualization(self, world_x, world_y):
        # Matplotlib görselleştirmesini güncelle
        self.lidar_points.set_offsets(np.c_[world_x, world_y])
        self.robot_position.set_offsets(np.c_[self.robot_x, self.robot_y])
        self.robot_path.set_data(self.path_x, self.path_y)
        
        # Robot footprint'i güncelle
        cos_yaw = np.cos(self.robot_yaw)
        sin_yaw = np.sin(self.robot_yaw)
        corner_x = self.robot_x - (self.robot_width/2) * cos_yaw + (self.robot_length/2) * sin_yaw
        corner_y = self.robot_y - (self.robot_width/2) * sin_yaw - (self.robot_length/2) * cos_yaw
        
        self.robot_footprint.set_xy((corner_x, corner_y))
        self.robot_footprint.set_angle(np.degrees(self.robot_yaw))
        
        # Harita görünümünü otomatik ayarla
        if len(world_x) > 0:
            all_x = np.concatenate([world_x, [self.robot_x]])
            all_y = np.concatenate([world_y, [self.robot_y]])
            
            x_min, x_max = np.min(all_x) - 1, np.max(all_x) + 1
            y_min, y_max = np.min(all_y) - 1, np.max(all_y) + 1
            
            # Minimum harita boyutu
            x_span = max(5, x_max - x_min)
            y_span = max(5, y_max - y_min)
            
            # Harita merkezini robot pozisyonuna göre ayarla
            center_x = self.robot_x
            center_y = self.robot_y
            
            plt.xlim(center_x - x_span/2, center_x + x_span/2)
            plt.ylim(center_y - y_span/2, center_y + y_span/2)
        
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedLidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Kapat
        if node.use_matplotlib:
            plt.ioff()
            plt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()