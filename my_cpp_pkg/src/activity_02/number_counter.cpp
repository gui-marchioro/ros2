#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter() : Node("number_counter")
    {
        m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        m_subscription = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounter::CallbackNumber, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Number counter node has been started");
        RCLCPP_INFO(this->get_logger(), "Counter value: '%ld'", m_count);
    }

private:
    int64_t m_count = 0;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_subscription;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;

    void CallbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        m_count += msg->data;
        RCLCPP_INFO(this->get_logger(), "Received number: '%ld'", msg->data);
        RCLCPP_INFO(this->get_logger(), "Counter value: '%ld'", m_count);
        PublishCount(m_count);
    }

    void PublishCount(const int64_t count)
    {
        auto message = example_interfaces::msg::Int64();
        message.data = count;
        m_publisher->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
