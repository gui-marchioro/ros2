#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number", 2)
        self.declare_parameter("time_period", 1.0)
        self._number = self.get_parameter("number").get_parameter_value().integer_value
        self._time_period = self.get_parameter("time_period").get_parameter_value().double_value
        self._number_publisher = self.create_publisher(Int64, "number", 10)
        self._number_timer = self.create_timer(self._time_period, self.publish_number)
        self.get_logger().info("Number publisher has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self._number
        self._number_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
