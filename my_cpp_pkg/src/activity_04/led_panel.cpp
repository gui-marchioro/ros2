#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        m_leds_state = {0, 0, 0};
        m_publisher = this->create_publisher<my_robot_interfaces::msg::LedPanelState>("led_panel_state", 10);
        m_service = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::handle_set_led, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "LED Panel has been started");
    }

private:
    std::array<std::uint8_t, 3> m_leds_state;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr m_publisher;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr m_service;

    void publish_led_state()
    {
        auto message = my_robot_interfaces::msg::LedPanelState();
        message.led_panel_state = m_leds_state;
        m_publisher->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published LED state: led0=%d, led1=%d, led2=%d", m_leds_state[0], m_leds_state[1], m_leds_state[2]);
    }

    void handle_set_led(
        const std::shared_ptr<my_robot_interfaces::srv::SetLed::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::SetLed::Response> response)
    {
        if (request->number > 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid LED number: %d", request->number);
            response->success = false;
            return;
        }
        m_leds_state[request->number] = request->state ? 1 : 0;
        RCLCPP_INFO(this->get_logger(), "Set LED %d to state %d", request->number, m_leds_state[request->number]);
        publish_led_state();
        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
