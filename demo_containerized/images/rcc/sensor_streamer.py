#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import random

class GridStreamer(Node):
    def __init__(self):
        super().__init__('grid_streamer')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(String, '/sensor/data', qos)
        self.timer = self.create_timer(0.5, self.timer_callback) # 2 Hz
        self.get_logger().info("Generic Grid Streamer started at 2Hz!")

    def timer_callback(self):
        msg = String()

        # Create a blank 3x3 grid (9 elements)
        grid = ['.'] * 9

        # Randomly place the 'X' in one of the slots (0 to 8)
        random_index = random.randint(0, 8)
        grid[random_index] = 'X'

        # Format it into a beautiful 3x3 matrix string with newlines
        msg.data = (
            f"\n"
            f"  {grid[0]}  {grid[1]}  {grid[2]}\n"
            f"  {grid[3]}  {grid[4]}  {grid[5]}\n"
            f"  {grid[6]}  {grid[7]}  {grid[8]}"
        )

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GridStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()