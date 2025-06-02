import sys
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSubscriber(Node):
    def __init__(self, port_name):
        super().__init__('keyboard_subscriber')
        self.get_logger().info(f"Keyboard Subscriber initialized on port: {port_name}")
        
        try:
            self.ser = serial.Serial(port_name, 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            raise SystemExit

        self.subscription = self.create_subscription(
            String,
            'keyboard_topic',
            self.listener_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.read_serial_data)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        try:
            self.ser.write(msg.data.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Write error: {e}")

    def read_serial_data(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Read from serial: {data}')
        except serial.SerialException as e:
            self.get_logger().error(f"Read error: {e}")

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Kullanım: ros2 run <paket_adi> keyboard_subscriber_node.py /dev/ttyUSB0")
        return

    port = sys.argv[1]
    node = KeyboardSubscriber(port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
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
        
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        self.ser.write(msg.data.encode('utf-8'))
    
    def read_serial_data(self):
        if self.ser.in_waiting > 0:  # Veri varsa
            data = self.ser.readline().decode('utf-8').strip()  # Satırı oku ve decode et
            self.get_logger().info(f'Read from serial: {data}')


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
