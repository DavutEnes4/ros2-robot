import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.get_logger().info("Keyboard Subscriber initialized.")
        
        # Serial portu başlat
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        
        self.subscription = self.create_subscription(
            String,  # Abone olunacak mesaj tipi
            'keyboard_topic',  # Publisher'ın gönderdiği topic adı
            self.listener_callback,  # Callback fonksiyonu
            10  # Queue size
        )
        self.subscription  # Subscription nesnesi referans olarak korunuyor

        # Seri porttan okuma yapmak için bir timer oluştur
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 0.1 saniyede bir tetiklenir

    def listener_callback(self, msg):
        # Abone olunduğunda gelen mesajı işleyin
        self.get_logger().info(f'Received: {msg.data}')
        # Gelen mesajı byte dizisine çevirip seri porta yaz
        self.ser.write(msg.data.encode('utf-8'))  # 'msg.data' stringini byte dizisine çevir ve yaz
    def read_serial_data(self):
        # Seri porttan veri oku
        if self.ser.in_waiting > 0:  # Veri varsa
            data = self.ser.readline().decode('utf-8').strip()  # Satırı oku ve decode et
            self.get_logger().info(f'Read from serial: {data}')
            # Gerekirse bu veriyi başka bir ROS 2 topic'e publish edebilirsiniz
            # Örneğin, yeni bir publisher tanımlayıp burada `publish` edebilirsiniz

    def destroy_node(self):
        # Node kapanmadan önce serial portu kapat
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = KeyboardSubscriber()

    rclpy.spin(subscriber_node)  # Subscriber düğümünü çalıştır

    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
