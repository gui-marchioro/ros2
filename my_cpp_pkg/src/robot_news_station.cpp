#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news")
    {
        this->declare_parameter("robot_name", "C3PO");
        m_robot_name = this->get_parameter("robot_name").as_string();
        m_publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RobotNewsStationNode::publish_news, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
    }

private:
    std::string m_robot_name;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void publish_news()
    {
        auto message = example_interfaces::msg::String();
        message.data = std::string("Hello from ") + m_robot_name + std::string(" robot");
        m_publisher->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
