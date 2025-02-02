#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HwPublisher : public rclcpp::Node
{
public:
    HwPublisher() : Node("nw_publisher")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hw_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HwPublisher::publishHwMessage, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status Node Online");
    }
private:
    
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void publishHwMessage()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 30;
        msg.are_motors_ready = true;
        msg.debug_message = "All Nominal";
        pub_->publish(msg);

    } 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HwPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}