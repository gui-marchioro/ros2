#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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
        self.add_post_set_parameters_callback(self.parameter_callback)

    def publish_number(self):
        msg = Int64()
        msg.data = self._number
        self._number_publisher.publish(msg)

    def parameter_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "number" and param.type_ == param.Type.INTEGER:
                self._number = param.value
                self.get_logger().info(f"Parameter 'number' changed to {self._number}")
            elif param.name == "time_period" and param.type_ == param.Type.DOUBLE:
                self._time_period = param.get_parameter_value().double_value
                self._number_timer.cancel()
                self._number_timer = self.create_timer(self._time_period, self.publish_number)
                self.get_logger().info(f"Parameter 'time_period' changed to {self._time_period}")


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
