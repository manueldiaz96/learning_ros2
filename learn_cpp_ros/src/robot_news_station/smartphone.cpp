#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("robot_news", 10, std::bind(&SmartphoneNode::callbackNews, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void callbackNews(std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->data);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}