
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>
#include <example_interfaces/srv/set_bool.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounter : public rclcpp::Node  // MODIFY NAME

{
public:
    NumberCounter() : Node("number_counter"), counter_(0)  // MODIFY NAME
    {
        sub_ = this->create_subscription<std_msgs::msg::Int64>("number", 10, std::bind(&NumberCounter::numberCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);
        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_number_count",
            std::bind(&NumberCounter::resetNumber, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "ResetServer started");
    }

private:
    unsigned int counter_ ;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;

    void numberCallback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        counter_++;
        auto numbers_counted = std_msgs::msg::Int64();
        numbers_counted.data = counter_;

        RCLCPP_INFO_STREAM(this->get_logger(), "I heard "<<msg->data);
        RCLCPP_INFO(this->get_logger(), "Current counter at %ld", numbers_counted.data);

        pub_->publish(numbers_counted);
    }

    void resetNumber(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                     const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Number Counter Reset Triggered";
            RCLCPP_INFO(this->get_logger(), "Number Counter Reset Triggered");
        }
        else{
            response->success = false;
            response->message = "Reset request received was False";
            RCLCPP_INFO(this->get_logger(), "Reset request received was False");
        }
     
    }
    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();  // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}