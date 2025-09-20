#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self._client = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a: int, b: int) -> None:
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available")
            return

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self._client.call_async(request)
        future.add_done_callback(partial(self.callback_response, request=request))

    def callback_response(self, future, request) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().info(
                f"Result of add_two_ints: a={request.a}, b={request.b}, sum={response.sum}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    node.call_add_two_ints(3, 7)
    node.call_add_two_ints(4, 5)
    node.call_add_two_ints(34, 56)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
