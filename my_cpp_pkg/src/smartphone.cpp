#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        m_subscription = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10, std::bind(&SmartphoneNode::CallbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone node has been started");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr m_subscription;

    void CallbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received news: '%s'", msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
