#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        this->declare_parameter("time_period", 1.0);
        m_number = this->get_parameter("number").as_int();
        double time_period = this->get_parameter("time_period").as_double();
        m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        m_timer = this->create_wall_timer(std::chrono::duration<double>(time_period), std::bind(&NumberPublisherNode::publish_number, this));
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
