#!usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyFirstNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self._counter = 0
        self.get_logger().info("Hello ROS2")
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self._counter += 1
        self.get_logger().info(f"Timer callback triggered. Count: {self._counter}")


def main(args=None):
    rclpy.init(args=args)

    node = MyFirstNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
