#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim_catch_them_all/msg/turtle.hpp"
#include "turtlesim_catch_them_all/msg/turtle_array.hpp"
#include "turtlesim_catch_them_all/srv/catch_turtle.hpp"
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
        m_timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TurtleControllerNode::MoveTowardsTarget, this));
        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        m_client = this->create_client<turtlesim_catch_them_all::srv::CatchTurtle>("catch_turtle");
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_subscription_pose;
    rclcpp::Subscription<turtlesim_catch_them_all::msg::TurtleArray>::SharedPtr m_subscription_turtles;
    rclcpp::TimerBase::SharedPtr m_timer;
    turtlesim::msg::Pose current_pose;
    turtlesim::msg::Pose target_pose;
    std::vector<turtlesim_catch_them_all::msg::Turtle> alive_turtles;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Client<turtlesim_catch_them_all::srv::CatchTurtle>::SharedPtr m_client;

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
        RCLCPP_DEBUG(this->get_logger(), "Current position (%.2f, %.2f)", current_pose.x, current_pose.y);
        double angle_to_target = atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x) - current_pose.theta;
        if (angle_to_target > M_PI) angle_to_target -= 2 * M_PI;
        if (angle_to_target < -M_PI) angle_to_target += 2 * M_PI;
        double distance_to_target = sqrt(pow(target_pose.x - current_pose.x, 2) + pow(target_pose.y - current_pose.y, 2));
        RCLCPP_DEBUG(this->get_logger(), "Distance to target: %.2f, Angle to target: %.2f", distance_to_target, angle_to_target);
        if (distance_to_target > 0.1) { // Threshold to consider "reached"
            if (distance_to_target < 0.5) {
                distance_to_target = 0.5;
            }
            message.linear.x = std::min(1.5, distance_to_target); // Cap speed
            message.angular.z = angle_to_target;
        } else {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reached target turtle at (%.2f, %.2f)", target_pose.x, target_pose.y);
            target_pose.x = 0; // Reset target
            target_pose.y = 0;
            while (!m_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
            }
            auto request = std::make_shared<turtlesim_catch_them_all::srv::CatchTurtle::Request>();
            request->name = alive_turtles[0].name; // Assuming we are catching the first turtle
            m_client->async_send_request(request,
                std::bind(&TurtleControllerNode::CaughtResponse, this, std::placeholders::_1));
        }
        RCLCPP_DEBUG(this->get_logger(), "Publishing cmd_vel - linear.x: %.2f, angular.z: %.2f",
                    message.linear.x, message.angular.z);
        m_publisher->publish(message);
    }

    void CaughtResponse(rclcpp::Client<turtlesim_catch_them_all::srv::CatchTurtle>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Caught turtle: %s", future.get()->success ? "Success" : "Failure");
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
