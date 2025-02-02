#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

#include <chrono>

class NumberPublisher : public rclcpp::Node  // MODIFY NAME
{
public:
    NumberPublisher() : Node("number_publisher")  // MODIFY NAME
    {
        RCLCPP_INFO_STREAM(get_logger(), "Publishing the same number now.");

        this->declare_parameter("number_to_publish", 42);
        this->declare_parameter("publish_frequency", 1.0);


        number_ = this->get_parameter("number_to_publish").as_int();
        double publish_freq_ = this->get_parameter("publish_frequency").as_double();

        pub_ = create_publisher<std_msgs::msg::Int64>("number", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds((int) (1000.0/publish_freq_)), std::bind(&NumberPublisher::sendNumberCallback, this));

        
    }

private:
    int number_ ;
    
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void sendNumberCallback()
    {
        auto message = std_msgs::msg::Int64();
        message.data = number_;

        pub_->publish(message);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();  // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}