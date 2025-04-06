#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
public:
    KalmanFilter(const std::string &name);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kalman_odom_publisher_;

    double mean_;
    double variance_;
    double last_imu_angular_z_;
    double last_enc_angular_z_;
    bool is_first_odom_;

    double motion_mean_;
    double motion_variance_;
    double measurement_variance_;

    nav_msgs::msg::Odometry kalman_odometry_;

    void odomCallback(const nav_msgs::msg::Odometry &odom);
    void imuCallback(const sensor_msgs::msg::Imu &imu);

    void measurementUpdate();
    void statePrediction();

};
#endif