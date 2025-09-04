#include "rclcpp/rclcpp.hpp"

class MyFirstNode : public rclcpp::Node
{
public:
    MyFirstNode() : Node("my_first_node"), m_count(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS2 from C++");
        m_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyFirstNode::TimerCallback, this)
        );
    }
private:
    void TimerCallback()
    {
        m_count++;
        RCLCPP_INFO(this->get_logger(), "Timer callback count: %d", m_count);
    }
    rclcpp::TimerBase::SharedPtr m_timer;
    int m_count;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyFirstNode>();

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}