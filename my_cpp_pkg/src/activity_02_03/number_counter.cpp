#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter() : Node("number_counter")
    {
        m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        m_subscription = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounter::CallbackNumber, this, std::placeholders::_1));
        m_service = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounter::ResetCounter, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Number counter node has been started");
        RCLCPP_INFO(this->get_logger(), "Counter value: '%ld'", m_count);
    }

private:
    int64_t m_count = 0;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_subscription;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr m_service;

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

    void ResetCounter(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                        std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
        std::string message = request->data ? "Counter reset to 0" : "Counter not reset";
        RCLCPP_INFO(this->get_logger(), message.c_str());
        request->data ? m_count = 0 : m_count = m_count;
        RCLCPP_INFO(this->get_logger(), "Counter value: '%ld'", m_count);
        PublishCount(m_count);
        response->success = true;
        response->message = message;
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
