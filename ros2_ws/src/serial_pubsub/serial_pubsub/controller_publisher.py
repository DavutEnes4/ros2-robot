import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time


class ControllerPublisher(Node):
    def __init__(self):
        super().__init__("controller_publisher")
        self.get_logger().info("Controller Publisher initialized.")
        self.publisher_ = self.create_publisher(String, "keyboard_topic", 10)
        self.last_publish_time = time.time()
        self.last_x, self.last_y = None, None  # Önceki eksen değerleri

    def listen_controller(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error(
                "No gamepad found. Please connect a gamepad and restart."
            )
            pygame.quit()
            return

        # İlk bağlı gamepad'i seç
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.get_logger().info(f"Gamepad connected: {joystick.get_name()}")

        try:
            x_axis, y_axis = 0, 0
            while rclpy.ok():  # ROS2'nin döngü durumu kontrolü
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:

                        # X ekseni
                        if event.axis == 0:
                            x_axis = round(event.value * 100, 2)

                        # Y ekseni
                        elif event.axis == 1:
                            y_axis = round(event.value * 100, 2)

                        # Verilerin yayımlanması için koşullar
                        current_time = time.time()
                        time_diff = current_time - self.last_publish_time

                        # Gönderme sıklığı ve yakınlık kontrolü
                        if (
                            (x_axis is not None and abs(x_axis - (self.last_x or 0)) > 5)
                            or (y_axis is not None and abs(y_axis - (self.last_y or 0)) > 5)
                        ) and time_diff >= 0.1:  # 100ms
                            msg = String()
                            msg.data = f"{x_axis}, {y_axis}"
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Publishing: {msg.data}")

                            # Durum güncellemeleri
                            self.last_publish_time = current_time
                            self.last_x, self.last_y = x_axis, y_axis

        except KeyboardInterrupt:
            self.get_logger().info("Exiting...")
        finally:
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerPublisher()
    try:
        node.listen_controller()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
