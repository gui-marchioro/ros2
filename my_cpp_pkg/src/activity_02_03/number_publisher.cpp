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
        m_timer = this->create_wall_timer(
            std::chrono::duration<double>(time_period),
            std::bind(&NumberPublisherNode::publish_number, this)
        );

        parameter_callback_handle_ =
            this->add_post_set_parameters_callback(
                std::bind(&NumberPublisherNode::parameter_callback, this, std::placeholders::_1)
            );

        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }

private:
    int64_t m_number;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    void publish_number()
    {
        auto message = example_interfaces::msg::Int64();
        message.data = m_number;
        m_publisher->publish(message);
    }

    void parameter_callback(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto & parameter : parameters)
        {
            if (parameter.get_name() == "number")
            {
                m_number = parameter.as_int();
                RCLCPP_INFO(this->get_logger(), "Number parameter changed to: %ld", m_number);
            }
            else if (parameter.get_name() == "time_period")
            {
                double time_period = parameter.as_double();
                m_timer->cancel();
                m_timer = this->create_wall_timer(
                    std::chrono::duration<double>(time_period),
                    std::bind(&NumberPublisherNode::publish_number, this)
                );
                RCLCPP_INFO(this->get_logger(), "Time period parameter changed to: %f seconds", time_period);
            }
        }
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
