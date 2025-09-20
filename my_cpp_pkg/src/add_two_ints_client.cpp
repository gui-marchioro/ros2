#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        m_client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void SendRequest(int64_t a, int64_t b)
    {
        while (!m_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        m_client->async_send_request(request,
             std::bind(&AddTwoIntsClientNode::CallbackResponse, this, std::placeholders::_1));
    }
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr m_client;
    void CallbackResponse(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Result: %ld", future.get()->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->SendRequest(5, 3);
    node->SendRequest(10, 20);
    node->SendRequest(100, 200);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
