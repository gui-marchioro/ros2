#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim_catch_them_all/msg/turtle.hpp"
#include "turtlesim_catch_them_all/msg/turtle_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        RCLCPP_INFO(this->get_logger(), "TurtleControllerNode has been started");
        m_subscription_pose = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::CallbackPose, this, std::placeholders::_1));
        m_subscription_turtles = this->create_subscription<turtlesim_catch_them_all::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::CallbackAliveTurtles, this, std::placeholders::_1));
        m_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleControllerNode::MoveTowardsTarget, this));
        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_subscription_pose;
    rclcpp::Subscription<turtlesim_catch_them_all::msg::TurtleArray>::SharedPtr m_subscription_turtles;
    rclcpp::TimerBase::SharedPtr m_timer;
    turtlesim::msg::Pose current_pose;
    turtlesim::msg::Pose target_pose;
    std::vector<turtlesim_catch_them_all::msg::Turtle> alive_turtles;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;

    void CallbackPose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Current Pose - x: %.2f, y: %.2f, theta: %.2f",
                    current_pose.x, current_pose.y, current_pose.theta);
    }

    void CallbackAliveTurtles(const turtlesim_catch_them_all::msg::TurtleArray::SharedPtr msg)
    {
        alive_turtles = msg->turtles;
        RCLCPP_INFO(this->get_logger(), "Alive Turtles:");
        for (const auto& turtle : alive_turtles) {
            RCLCPP_INFO(this->get_logger(), " - Name: %s, x: %.2f, y: %.2f",
                        turtle.name.c_str(), turtle.x, turtle.y);
        }
        if (!alive_turtles.empty()) {
            // For simplicity, target the first turtle in the list
            target_pose.x = alive_turtles[0].x;
            target_pose.y = alive_turtles[0].y;
            RCLCPP_INFO(this->get_logger(), "Targeting Turtle - Name: %s, x: %.2f, y: %.2f",
                        alive_turtles[0].name.c_str(), target_pose.x, target_pose.y);
        }
    }

    void MoveTowardsTarget()
    {
        if (target_pose.x == 0 && target_pose.y == 0) {
            // No target set
            return;
        }

        auto message = geometry_msgs::msg::Twist();

        double angle_to_target = atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x);
        double distance_to_target = sqrt(pow(target_pose.x - current_pose.x, 2) + pow(target_pose.y - current_pose.y, 2));

        if (distance_to_target > 0.1) { // Threshold to consider "reached"
            message.linear.x = std::min(1.0, distance_to_target); // Cap speed to 1.0
            message.angular.z = angle_to_target - current_pose.theta;
        } else {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reached target turtle at (%.2f, %.2f)", target_pose.x, target_pose.y);
            target_pose.x = 0; // Reset target
            target_pose.y = 0;
        }

        m_publisher->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
