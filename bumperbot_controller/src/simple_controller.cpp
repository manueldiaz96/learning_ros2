#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>

SimpleController::SimpleController(const std::string &name) : Node(name)
{
    this->declare_parameter("wheel_radius", 0.033);
    this->declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    // Create a QoS profile with a history depth of 10
    pub_wheel_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velocityCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Using wheel radius: %f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Using wheel separation: %f", wheel_separation_);

    // Initialize the speed conversion matrix
    speed_conversion_matrix_ << wheel_radius_ / 2.0, wheel_radius_ / 2.0,
                                 wheel_radius_ / wheel_separation_, -wheel_radius_ / wheel_separation_;

    RCLCPP_INFO_STREAM(this->get_logger(), "The conversion matrix is \n" << speed_conversion_matrix_);
}

void SimpleController::velocityCallback(const geometry_msgs::msg::TwistStamped & msg)
{
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion_matrix_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;

    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    pub_wheel_cmd_->publish(wheel_speed_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
