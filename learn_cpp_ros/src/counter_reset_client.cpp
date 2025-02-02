#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class ResetCounterClient : public rclcpp::Node
{
public:
    ResetCounterClient() : Node("reset_counter_client")
    {
        thread_ = std::thread(std::bind(&ResetCounterClient::callResetService, this, true));
    }
private:
    std::thread thread_;

    void callResetService(bool reset_counter)
    {
        auto client = this->create_client<example_interfaces::srv::SetBool>("reset_number_count");
    
        // Wait until the server is available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to start...");
        }
    
        // Create request
        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        // Set request parameters
        request->data = reset_counter;
        // Send async request
        auto future = client->async_send_request(request);
    
        try
        {
            auto response = future.get();
            // Handle response
            RCLCPP_INFO_STREAM(this->get_logger(), "Service call succeeded | Message: " << response->message);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetCounterClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}