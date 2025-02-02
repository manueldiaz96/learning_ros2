#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher_cpp"), counter_(0)
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

        //  Remember to add the pointer handler  &
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SimplePublisher::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

private:
    unsigned int counter_ ; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2 - Message number: " + std::to_string(counter_);
        counter_++;

        pub_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 

    return 0;
}