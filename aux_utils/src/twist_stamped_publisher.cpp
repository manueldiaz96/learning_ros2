#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistStampedPubWithHeader : public rclcpp::Node
{
public:
    TwistStampedPubWithHeader() : Node("twist_header_node")
    {
        this->declare_parameter("in_topic_name", "/cmd_vel_unstamped");
        this->declare_parameter("out_topic_name", "/cmd_vel");
        
        in_topic_name = this->get_parameter("in_topic_name").as_string();
        out_topic_name = this->get_parameter("out_topic_name").as_string();

        twist_stamped_pub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                                                                    in_topic_name,
                                                                    10,
                                                                    std::bind(&TwistStampedPubWithHeader::addHeaderToTwistStamped, 
                                                                              this,
                                                                              std::placeholders::_1 
                                                                            ));

        header_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(out_topic_name, 10);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Stamping Twist Messages from \"" << in_topic_name << "\" and publishing to \"" << out_topic_name << "\"");
    }
private:
    std::string in_topic_name, out_topic_name;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr header_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub;

    void addHeaderToTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg){

        geometry_msgs::msg::TwistStamped twist_stamped;

        twist_stamped.header.stamp = this->get_clock()->now(); 
        twist_stamped.header.frame_id = msg->header.frame_id;  

        twist_stamped.twist = msg->twist;

        header_pub_->publish(twist_stamped);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistStampedPubWithHeader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}