#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self._counter = 0
        self._number_count_publisher = self.create_publisher(
            Int64, "number_count", 10)
        self._number_subscriber = self.create_subscription(
            Int64, "number", self.callback_number, 10)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg: Int64):
        self._counter += msg.data
        new_msg = Int64()
        new_msg.data = self._counter
        self._number_count_publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
