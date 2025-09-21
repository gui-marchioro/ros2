#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        m_robot_name = "C3PO";
        m_publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisherNode::publish_hw_status, this));
        RCLCPP_INFO(this->get_logger(), "HardwareStatusPublisher has been started");
    }

private:
    std::string m_robot_name;
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void publish_hw_status()
    {
        auto message = my_robot_interfaces::msg::HardwareStatus();
        message.temperature = 45.0;
        message.are_motors_ready = true;
        message.debug_message = "Nothing to report";
        m_publisher->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
