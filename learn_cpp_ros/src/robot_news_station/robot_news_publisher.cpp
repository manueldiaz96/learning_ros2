#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NewsPublisher : public rclcpp::Node
{
public:
    NewsPublisher() : Node("robot_news_c3po")
    {
        this->declare_parameter<std::string>("robot_name", "C3PO");
        robot_name = this->get_parameter("robot_name").as_string();

        pub_ = this->create_publisher<std_msgs::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NewsPublisher::PublishNews, this));
    }
private:
    std::string robot_name;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void PublishNews(){
        auto message = std_msgs::msg::String();
        message.data = "Hello, here is " + robot_name;
        pub_->publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(), message.data);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NewsPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}