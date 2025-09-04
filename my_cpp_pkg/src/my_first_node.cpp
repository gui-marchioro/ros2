#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("my_first_node");
    RCLCPP_INFO(node->get_logger(), "Hello, ROS2 from C++17");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}