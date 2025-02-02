#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LED_Node : public rclcpp::Node
{
public:
    LED_Node() : Node("led_node")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);

        // CLI ->  -p led_array:="['off','off','on']"

        this->declare_parameter<std::vector<std::string>>("led_array", {"off", "off", "off"});

        this->get_parameter("led_array", led_array);

        // These are using lambda functions instead of std::bind
        // it is more readable and does not need to use placeholders
        // below are their equivalents with std::bind

        service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led", [this](const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                                                                                                  my_robot_interfaces::srv::SetLed::Response::SharedPtr response) {
                                                                                                  setLedValue(request, response);});
        // service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led", std::bind(&LED_Node::setLedValue, this, _1, _2));

        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this](){publishLedStates();});
        // timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LED_Node:publishLedStates, this));_
    }
private:
    std::vector<std::string> led_array;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setLedValue(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request_,
                           my_robot_interfaces::srv::SetLed::Response::SharedPtr response_)
    {
        
        led_array[request_->led_index] = request_->status;

        response_->success = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "LED " << std::to_string(request_->led_index) << " set to " << request_->status);
        
    }

    void publishLedStates(){
        auto ledMessage = my_robot_interfaces::msg::LedStates();
        ledMessage.led_1_status = led_array[0];
        ledMessage.led_2_status = led_array[1];
        ledMessage.led_3_status = led_array[2];

        pub_->publish(ledMessage);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LED_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}