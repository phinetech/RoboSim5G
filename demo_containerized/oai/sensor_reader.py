#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SensorReceiver(Node):
    def __init__(self):
        super().__init__('grid_receiver')

        # Exakt dasselbe QoS-Profil (Reliable + Volatile)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            String,
            '/sensor/data',
            self.listener_callback,
            qos
        )

        # Einmaliges Löschen des Bildschirms beim Start
        print("\033[2J", end="", flush=True)
        self.get_logger().info(
            "Reader is ready. Wait for 5G-Grid-Data..."
        )

    def listener_callback(self, msg):
        # \033[H  -> Bewegt den Cursor ganz nach oben links im Terminal
        # \033[J  -> Löscht alles ab der aktuellen Cursorposition bis unten hin
        # end=""  -> Verhindert, dass print() automatisch eine extra Leerzeile dranhängt
        output = (
            f"\033[H\033[J--- [  LIVE DEMO FEED ] ---\n"
            f"{msg.data}\n\n"
            f"---------------------------\n"
            f"[Ctrl+C] Quit"
        )
        print(output, end="", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = SensorReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()