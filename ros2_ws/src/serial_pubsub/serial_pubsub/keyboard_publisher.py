import rclpy
from rclpy.node import Node
import sys
import termios
import tty
from std_msgs.msg import String

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.get_logger().info("Keyboard Publisher initialized. Press any key...")
        self.publisher_ = self.create_publisher(String, 'keyboard_topic', 10)

    def listen_keyboard(self):
        # Terminal ayarlarını değiştir
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)  # Tek bir karakter oku
                if key:  # Tuş basılmışsa işlem yap
                    self.get_logger().info(f"Key pressed: {key}")
                    msg = String()
                    msg.data = key  # Mesajı direkt tuş girdisi olarak ata
                    self.publisher_.publish(msg)
                    
                    # 'q' tuşuna basıldığında çıkış yap
                    if key.lower() == 'q':
                        self.get_logger().info("Exiting...")
                        break
        finally:
            # Eski terminal ayarlarını geri yükle
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.listen_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
