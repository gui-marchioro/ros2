#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        RCLCPP_INFO(this->get_logger(), "TurtleSpawnerNode has been started");
        m_client = this->create_client<turtlesim::srv::Spawn>("spawn");
        m_base_turtle_name = "spawned_turtle";
        spawn_new_turtle();
        m_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TurtleSpawnerNode::spawn_new_turtle, this));
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr m_client;
    std::string m_base_turtle_name;
    std::uint16_t m_spawner_counter = 0;
    rclcpp::TimerBase::SharedPtr m_timer;

    void spawn_new_turtle()
    {
        m_spawner_counter++;
        std::string turtle_name = m_base_turtle_name + std::to_string(m_spawner_counter);
        spawn_turtle(turtle_name);
    }

    void spawn_turtle(const std::string &name)
    {
        RCLCPP_INFO(this->get_logger(), "Spawning turtle: %s", name.c_str());
        float x = get_random_position();
        float y = get_random_position();
        float theta = get_random_rotation();

        while (!m_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        m_client->async_send_request(request,
             std::bind(&TurtleSpawnerNode::spawn_response, this, std::placeholders::_1));
    }

    float get_random_position()
    {
        // Generate a random number for turtle position
        float max_position = 11.0f;
        return static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * max_position;
    }

    float get_random_rotation()
    {
        // Generate a random number for turtle angle
        float pi = 3.14f;
        return static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * pi;
    }

    void spawn_response(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", future.get()->name.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
