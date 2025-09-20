#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    """
    Testing this service:
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
    """
    def __init__(self):
        super().__init__("add_two_ints_server")
        self._server = self.create_service(
            AddTwoInts, "add_two_ints", self.add_two_ints_callback
        )
        self.get_logger().info("Service AddTwoIntsServer has been started")

    def add_two_ints_callback(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        response.sum = request.a + request.b
        self.get_logger().info(
            f"Incoming request: a={request.a}, b={request.b}, sum={response.sum}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
