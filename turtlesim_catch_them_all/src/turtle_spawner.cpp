#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim_catch_them_all/msg/turtle.hpp"
#include "turtlesim_catch_them_all/msg/turtle_array.hpp"
#include "turtlesim_catch_them_all/srv/catch_turtle.hpp"

using namespace std::placeholders;
using namespace turtlesim_catch_them_all::msg;
using namespace turtlesim_catch_them_all::srv;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        RCLCPP_INFO(this->get_logger(), "TurtleSpawnerNode has been started");
        m_client_spawn = this->create_client<turtlesim::srv::Spawn>("spawn");
        m_client_kill = this->create_client<turtlesim::srv::Kill>("kill");
        m_base_turtle_name = "spawned_turtle";
        m_timer = this->create_wall_timer(std::chrono::seconds(5), std::bind(&TurtleSpawnerNode::spawn_new_turtle, this));
        m_publisher = this->create_publisher<TurtleArray>("alive_turtles", 10);
        m_service = this->create_service<CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawnerNode::catch_turtle, this, _1, _2));
        spawn_new_turtle();
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr m_client_spawn;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr m_client_kill;
    std::string m_base_turtle_name;
    std::uint16_t m_spawner_counter = 0;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<Turtle> m_spawned_turtles;
    rclcpp::Publisher<TurtleArray>::SharedPtr m_publisher;
    rclcpp::Service<CatchTurtle>::SharedPtr m_service;

    void spawn_new_turtle()
    {
        m_spawner_counter++;
        std::string turtle_name = m_base_turtle_name + std::to_string(m_spawner_counter);
        auto turtle = spawn_turtle(turtle_name);
        m_spawned_turtles.push_back(turtle);
        publish_alive_turtles();
    }

    Turtle spawn_turtle(const std::string &name)
    {
        RCLCPP_INFO(this->get_logger(), "Spawning turtle: %s", name.c_str());
        float x = get_random_position();
        float y = get_random_position();
        float theta = get_random_rotation();

        while (!m_client_spawn->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto turtle = Turtle();
        turtle.name = name;
        turtle.x = x;
        turtle.y = y;

        m_client_spawn->async_send_request(request,
             std::bind(&TurtleSpawnerNode::spawn_response, this, std::placeholders::_1));
        return turtle;
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

    void publish_alive_turtles()
    {
        TurtleArray msg;
        msg.turtles = m_spawned_turtles;
        m_publisher->publish(msg);
    }

    void catch_turtle(
        const std::shared_ptr<CatchTurtle::Request> request,
        std::shared_ptr<CatchTurtle::Response> response)
    {
        while (!m_client_kill->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = request->name;
        
        m_client_kill->async_send_request(kill_request,
             std::bind(&TurtleSpawnerNode::kill_response, this, _1));
        RCLCPP_INFO(this->get_logger(), "Catching turtle: %s", request->name.c_str());
        response->success = true;
        for (auto it = m_spawned_turtles.begin(); it != m_spawned_turtles.end(); ++it) {
            if (it->name == request->name) {
                m_spawned_turtles.erase(it);
                break;
            }
        }
        publish_alive_turtles();
    }

    void kill_response(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Turtle killed");
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
