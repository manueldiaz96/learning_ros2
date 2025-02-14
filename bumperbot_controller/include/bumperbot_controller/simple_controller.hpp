#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string &name);

private:
    void velocityCallback(const geometry_msgs::msg::TwistStamped & msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_cmd_;

    double wheel_radius_;
    double wheel_separation_;

    Eigen::Matrix2d speed_conversion_matrix_;
};

#endif // SIMPLE_CONTROLLER_HPP
