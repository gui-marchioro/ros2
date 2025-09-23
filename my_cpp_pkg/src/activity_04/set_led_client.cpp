#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("set_led_client_no_oop");

    auto client = node->create_client<my_robot_interfaces::srv::SetLed>("set_led");
    while (!client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
    request->number = 2;
    request->state = true;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Success %d ", result_future.get()->success);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
    }
    
    rclcpp::shutdown();
    return 0;
}