#include "bumperbot_controller/noisy_controller.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <random>

NoisyController::NoisyController(const std::string &name) : 
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
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 
                                                                                10, 
                                                                                std::bind(&NoisyController::jointCallback, 
                                                                                            this, 
                                                                                            std::placeholders::_1
                                                                                        )
                                                                              );

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_ekf";

    odometry_msg_.pose.pose.orientation.x = 0.0;
    odometry_msg_.pose.pose.orientation.y = 0.0;
    odometry_msg_.pose.pose.orientation.z = 0.0;
    odometry_msg_.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Using wheel radius: %f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Using wheel separation: %f", wheel_separation_);


}

void NoisyController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);

    double delta_pos_left_wheel = wheel_encoder_left - left_wheel_previous_position_;
    double delta_pos_right_wheel = wheel_encoder_right - right_wheel_previous_position_;

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

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();

    transform_stamped_.header.stamp = odometry_msg_.header.stamp;

    odometry_pub_->publish(odometry_msg_);
    transform_broadcaster_->sendTransform(transform_stamped_);


}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
