#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        m_service = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::handle_add_two_ints, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service AddTwoIntsServer has been started");
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr m_service;

    void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld, sum=%ld", request->a, request->b, response->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
