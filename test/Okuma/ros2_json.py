import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import time

class KeyboardTopicNode(Node):
    def __init__(self):
        super().__init__("keyboard_topic_node")

        # ROS 2 Publisher oluştur
        self.publisher_ = self.create_publisher(String, "keyboard_topic", 10)

        # Seri portu açmaya çalış
        try:
            self.ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
            self.get_logger().info("Seri port bağlantısı başarılı!")
        except serial.SerialException:
            self.get_logger().error("Seri port açılırken hata oluştu!")
            self.ser = None

        # Zamanlayıcı başlat (50 ms periyoduyla çalışacak)
        self.timer = self.create_timer(0.05, self.read_serial_data)

    def read_serial_data(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                # Seri porttan gelen veriyi oku
                data = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Read from serial: {data}")

                # Mesajı ROS 2 topiğine yayınla
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)

                # Gelen veriyi log dosyasına kaydet
                with open("log.json", "a", encoding="utf-8") as log_file:
                    json.dump({"received": data, "timestamp": time.time()}, log_file)
                    log_file.write("\n")

                # Eğer gelen veri "veri" ise, JSON dosyasını oku ve seri porta yaz
                if data == "veri":
                    try:
                        with open("veri.json", "r", encoding="utf-8") as f:
                            veri = json.load(f)
                        json_data = json.dumps(veri)
                        self.ser.write(json_data.encode('utf-8'))
                        self.get_logger().info(f"Write to serial: {json_data}")
                    except (json.JSONDecodeError, FileNotFoundError):
                        self.get_logger().error("veri.json okunamadı!")

            except Exception as e:
                self.get_logger().error(f"Seri port okuma hatası: {e}")
mv src/keyboard_input/keyboard_input/topic.py src/keyboard_input/keyboard_input/nano.py
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTopicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
