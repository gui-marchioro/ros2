#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        m_number = 5;
        m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::publish_number, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }

private:
    int64_t m_number;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void publish_number()
    {
        auto message = example_interfaces::msg::Int64();
        message.data = m_number;
        m_publisher->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
