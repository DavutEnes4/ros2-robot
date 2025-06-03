import sys
import re
import serial
import rclpy
from rclpy.node import Node


class MeshMessageForwarder(Node):
    """Seri hatta gelen ESP‑Mesh çıktısından *sadece mesaj içeriğini* ayıklar,
    ikinci seri porta iletir **ve** işlenme durumunu da yazar.

    Kabul edilen satır formatları:
      [123456] size gelen mesaj: Merhaba
      Node 897'ya mesaj gönderildi: Merhaba
    Çıktı formatı (çıkış portuna iki satır gönderilir):
      Merhaba
      STATUS: FORWARDED
    """

    _PATTERN_RECV = re.compile(r"^\[\d+\] size gelen mesaj: (.+)")
    _PATTERN_SENT = re.compile(r"^Node \d+'ya mesaj g\u00f6nderildi: (.+)")

    def __init__(self, port_in: str, port_out: str, baudrate: int = 115200):
        super().__init__('mesh_message_forwarder')
        self.get_logger().info(f"Başladı: {port_in} → {port_out}")

        try:
            self.ser_in = serial.Serial(port_in, baudrate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Giriş portu hatası: {e}")
            raise SystemExit

        try:
            self.ser_out = serial.Serial(port_out, baudrate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Çıkış portu hatası: {e}")
            self.ser_in.close()
            raise SystemExit

        # 100 Hz okuma d\u00f6ng\u00fcs\u00fc
        self.timer = self.create_timer(0.01, self._process_serial)

    # ---------------------------------------------------------------
    @staticmethod
    def _extract_payload(line: str):
        """Seri satırdan mesajı döndürür; bulunamazsa None."""
        m = MeshMessageForwarder._PATTERN_RECV.match(line)
        if not m:
            m = MeshMessageForwarder._PATTERN_SENT.match(line)
        return m.group(1).strip() if m else None

    # ---------------------------------------------------------------
    def _process_serial(self):
        if self.ser_in.in_waiting == 0:
            return

        raw = self.ser_in.readline().decode('utf-8', errors='ignore').strip()
        if not raw:
            return

        payload = self._extract_payload(raw)
        if payload is None:
            return

        status_line = "STATUS: FORWARDED"

        # Çıkış portuna iki satır: mesaj + durum
        if self.ser_out.is_open:
            try:
                self.ser_out.write((payload + '\n').encode('utf-8'))
                self.ser_out.write((status_line + '\n').encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Çıkış portu yazma hatası: {e}")

        self.get_logger().info(f"Mesaj iletildi → '{payload}'")

    # ---------------------------------------------------------------
    def destroy_node(self):
        if self.ser_in.is_open:
            self.ser_in.close()
        if self.ser_out.is_open:
            self.ser_out.close()
        super().destroy_node()


# -------------------------------------------------------------------
# main()
# -------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Kullanım: ros2 run <paket_adi> mesh_message_forwarder.py /dev/ttyUSB0 /dev/ttyUSB1")
        return

    port_in = sys.argv[1]
    port_out = sys.argv[2]

    node = MeshMessageForwarder(port_in, port_out)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
