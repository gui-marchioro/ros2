#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
        m_battery_state = 100;
        m_is_charging = false;
        m_timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&BatteryNode::UpdateBatteryState, this));
        m_client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        RCLCPP_INFO(this->get_logger(), "Battery has been started");
    }

private:
    uint8_t m_battery_state;
    bool m_is_charging ;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr m_client;

    void UpdateBatteryState()
    {
        if (m_battery_state > 0 && !m_is_charging)
        {
            m_battery_state -= 5;
        }
        else
        {
            m_battery_state += 3;
        }

        if (m_battery_state <= 0)
        {
            m_is_charging = true;
            m_battery_state = 0;
            RCLCPP_INFO(this->get_logger(), "Battery depleted! Starting to charge...");
            // Turn on LED 0 to indicate charging
            auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
            request->number = 0;
            request->state = true;
            if (m_client->wait_for_service(std::chrono::seconds(1))) {
                m_client->async_send_request(request);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to call service set_led");
            }
        }
        else if (m_battery_state >= 100)
        {
            m_battery_state = 100;
            m_is_charging = false;
            RCLCPP_INFO(this->get_logger(), "Battery fully charged! Stopping charge...");
            // Turn off LED 0
            auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
            request->number = 0;
            request->state = false;
            if (m_client->wait_for_service(std::chrono::seconds(1))) {
                m_client->async_send_request(request);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to call service set_led");
            }
        }
        RCLCPP_INFO(this->get_logger(), "Battery state: %d%%", m_battery_state);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
