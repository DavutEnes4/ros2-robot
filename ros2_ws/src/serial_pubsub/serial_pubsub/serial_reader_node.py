import sys
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SerialReader(Node):
    def __init__(self, port_name):
        super().__init__('serial_reader_node')
        self.get_logger().info(f'Serial Reader started on port: {port_name}')
        
        try:
            self.ser = serial.Serial(port_name, 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial port error: {e}')
            raise SystemExit

        self.publisher = self.create_publisher(String, 'keyboard_topic', 10)
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10Hz

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: {line}')

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Seri port argümanla verilmeli
    if len(sys.argv) < 2:
        print("Kullanım: ros2 run <paket_adi> serial_reader_node.py /dev/ttyUSB0")
        return

    port = sys.argv[1]
    node = SerialReader(port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
