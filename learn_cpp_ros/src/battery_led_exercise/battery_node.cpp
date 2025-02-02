#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), batteryLevel(100.0)
    {
        launch_battery_monitor = std::thread(&BatteryNode::checkBatteryLevel, this);
    }
private:
    float batteryLevel;
    std::thread launch_battery_monitor;
    std::array<bool, 4> trigger_states = {false, false, false, false};
    void checkBatteryLevel()
    {
         while (rclcpp::ok()){
            batteryLevel -= 1.0;

            if (batteryLevel >= 60 && batteryLevel < 90 && !trigger_states[2])
            {   
                callLedService(2, "on");
                trigger_states[2] = true;
                
            }
            else if (batteryLevel > 30 && batteryLevel < 60 && !trigger_states[1])
            {
                callLedService(1, "on");
                trigger_states[1] = true;
            }
            else if (batteryLevel <= 30 && !trigger_states[0])
            {
                callLedService(0, "on");
                trigger_states[0] = true;
            }
            else if (!trigger_states[3])
            {
                callLedService(0, "off");
                callLedService(1, "off");
                callLedService(2, "off");
                trigger_states[3] = true;
            }

            if (batteryLevel == 0){
                batteryLevel = 100.0;
                trigger_states = {false, false, false, false};
            }

            RCLCPP_INFO(this->get_logger(),"Battery Level at %f", batteryLevel);

            std::this_thread::sleep_for(std::chrono::seconds(1));
         }
    }
    void callLedService(int led_index, std::string led_status)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
    
        // Wait until the server is available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to start...");
        }
    
        // Create request
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        // Set request parameters
        request->led_index = led_index;
        request->status = led_status;

        // Send async request
        auto future = client->async_send_request(request);
    
        try
        {
            auto response = future.get();
            // Handle response
            RCLCPP_INFO_STREAM(this->get_logger(), "Request for LED " << std::to_string(request->led_index) << " to be " << request->status);
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
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}