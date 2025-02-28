#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.hpp>

SimpleController::SimpleController(const std::string &name) : 
    Node(name), 
    left_wheel_previous_position_(0.0),
    right_wheel_previous_position_(0.0),
    x_(0.0),
    y_(0.0),
    theta_(0.0)
{
    previous_time_ = this->get_clock()->now();

    this->declare_parameter("wheel_radius", 0.033);
    this->declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    // Create a QoS profile with a history depth of 10
    pub_wheel_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 
                                                                                10, 
                                                                                std::bind(&SimpleController::velocityCallback, 
                                                                                          this, 
                                                                                          std::placeholders::_1
                                                                                        )
                                                                              );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 
                                                                                10, 
                                                                                std::bind(&SimpleController::jointCallback, 
                                                                                            this, 
                                                                                            std::placeholders::_1
                                                                                        )
                                                                              );

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);

    odometry_msg_.header.frame_id = "odom";
    odometry_msg_.child_frame_id = "base_footprint";
    odometry_msg_.pose.pose.orientation.x = 0.0;
    odometry_msg_.pose.pose.orientation.y = 0.0;
    odometry_msg_.pose.pose.orientation.z = 0.0;
    odometry_msg_.pose.pose.orientation.w = 1.0;

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

void SimpleController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    double delta_pos_left_wheel = msg.position.at(1) - left_wheel_previous_position_;
    double delta_pos_right_wheel = msg.position.at(0) - right_wheel_previous_position_;

    rclcpp::Time msg_time = msg.header.stamp;

    rclcpp::Duration dt = msg_time - previous_time_;

    left_wheel_previous_position_ = msg.position.at(1);
    right_wheel_previous_position_ = msg.position.at(0);
    previous_time_ = msg_time;

    double phi_left = delta_pos_left_wheel / dt.seconds();
    double phi_right = delta_pos_right_wheel / dt.seconds();

    double linear_vel = (wheel_radius_*phi_right + wheel_radius_*phi_left)/2;
    double angular_vel = ((wheel_radius_/wheel_separation_)*phi_right - (wheel_radius_/wheel_separation_)*phi_left);

    double d_position = ((wheel_radius_ * delta_pos_right_wheel) + (wheel_radius_ * delta_pos_left_wheel))/2;
    double d_theta = ((wheel_radius_/wheel_separation_) * delta_pos_right_wheel - (wheel_radius_/wheel_separation_) * delta_pos_left_wheel);
    
    theta_ += d_theta;
    x_ += d_position * cos(theta_);
    y_ += d_position * sin(theta_); 

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odometry_msg_.pose.pose.orientation.x = q.x();
    odometry_msg_.pose.pose.orientation.y = q.y();
    odometry_msg_.pose.pose.orientation.z = q.z();
    odometry_msg_.pose.pose.orientation.w = q.w();

    odometry_msg_.pose.pose.position.x = x_;
    odometry_msg_.pose.pose.position.y = y_;

    odometry_msg_.header.stamp = this->get_clock()->now();

    odometry_msg_.twist.twist.linear.x = linear_vel;
    odometry_msg_.twist.twist.angular.z = angular_vel;

    odometry_pub_->publish(odometry_msg_);


}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
