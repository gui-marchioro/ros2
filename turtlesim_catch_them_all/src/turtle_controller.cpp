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
        this->declare_parameter("catch_closest_turtle_first", true);
        m_closest_turtle_first = this->get_parameter("catch_closest_turtle_first").as_bool();
        m_subscription_pose = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::CallbackPose, this, std::placeholders::_1));
        m_subscription_turtles = this->create_subscription<turtlesim_catch_them_all::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::CallbackAliveTurtles, this, std::placeholders::_1));
        m_timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TurtleControllerNode::MoveTowardsTarget, this));
        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        m_client = this->create_client<turtlesim_catch_them_all::srv::CatchTurtle>("catch_turtle");
    }

private:
    bool m_closest_turtle_first;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_subscription_pose;
    rclcpp::Subscription<turtlesim_catch_them_all::msg::TurtleArray>::SharedPtr m_subscription_turtles;
    rclcpp::TimerBase::SharedPtr m_timer;
    turtlesim::msg::Pose m_current_pose;
    turtlesim::msg::Pose m_target_pose;
    std::string m_target_turtle;
    std::vector<turtlesim_catch_them_all::msg::Turtle> m_alive_turtles;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Client<turtlesim_catch_them_all::srv::CatchTurtle>::SharedPtr m_client;

    void CallbackPose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        m_current_pose = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Current Pose - x: %.2f, y: %.2f, theta: %.2f",
                    m_current_pose.x, m_current_pose.y, m_current_pose.theta);
    }

    void CallbackAliveTurtles(const turtlesim_catch_them_all::msg::TurtleArray::SharedPtr msg)
    {
        m_alive_turtles = msg->turtles;
        RCLCPP_INFO(this->get_logger(), "Alive Turtles:");
        for (const auto& turtle : m_alive_turtles) {
            RCLCPP_INFO(this->get_logger(), " - Name: %s, x: %.2f, y: %.2f",
                        turtle.name.c_str(), turtle.x, turtle.y);
        }
        if (m_closest_turtle_first) {
            TargetClosestTurtle();
        } else {
            TargetFirstTurtle();
        }
    }

    void TargetClosestTurtle()
    {
        if (m_alive_turtles.empty()) {
            return;
        }
        double min_distance = std::numeric_limits<double>::max();
        turtlesim_catch_them_all::msg::Turtle closest_turtle;
        for (const auto& turtle : m_alive_turtles) {
            double distance = sqrt(pow(turtle.x - m_current_pose.x, 2) + pow(turtle.y - m_current_pose.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_turtle = turtle;
            }
        }
        m_target_pose.x = closest_turtle.x;
        m_target_pose.y = closest_turtle.y;
        m_target_turtle = closest_turtle.name;
        RCLCPP_INFO(this->get_logger(), "Closest Turtle - Name: %s, x: %.2f, y: %.2f",
                    m_target_turtle.c_str(), m_target_pose.x, m_target_pose.y);
    }

    void TargetFirstTurtle()
    {
        if (m_alive_turtles.empty()) {
            return;
        }
        m_target_pose.x = m_alive_turtles[0].x;
        m_target_pose.y = m_alive_turtles[0].y;
        m_target_turtle = m_alive_turtles[0].name;
        RCLCPP_INFO(this->get_logger(), "First Turtle - Name: %s, x: %.2f, y: %.2f",
                    m_target_turtle.c_str(), m_target_pose.x, m_target_pose.y);
    }

    void MoveTowardsTarget()
    {
        if (m_target_pose.x == 0 && m_target_pose.y == 0) {
            // No target set
            return;
        }

        auto message = geometry_msgs::msg::Twist();
        RCLCPP_DEBUG(this->get_logger(), "Current position (%.2f, %.2f)", m_current_pose.x, m_current_pose.y);
        double angle_to_target = atan2(m_target_pose.y - m_current_pose.y, m_target_pose.x - m_current_pose.x) - m_current_pose.theta;
        if (angle_to_target > M_PI) angle_to_target -= 2 * M_PI;
        if (angle_to_target < -M_PI) angle_to_target += 2 * M_PI;
        double distance_to_target = sqrt(pow(m_target_pose.x - m_current_pose.x, 2) + pow(m_target_pose.y - m_current_pose.y, 2));
        RCLCPP_DEBUG(this->get_logger(), "Distance to target: %.2f, Angle to target: %.2f", distance_to_target, angle_to_target);
        if (distance_to_target > 0.1) { // Threshold to consider "reached"
            message.linear.x = std::min(1.5, distance_to_target); // Cap speed
            message.angular.z = angle_to_target;
        } else {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reached target turtle at (%.2f, %.2f)", m_target_pose.x, m_target_pose.y);
            m_target_pose.x = 0; // Reset target
            m_target_pose.y = 0;
            while (!m_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
            }
            auto request = std::make_shared<turtlesim_catch_them_all::srv::CatchTurtle::Request>();
            request->name = m_target_turtle;
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
