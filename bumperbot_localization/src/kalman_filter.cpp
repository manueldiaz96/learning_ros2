#include "bumperbot_localization/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const std::string &name):
    Node(name),
    mean_(0.0),
    variance_(1000.0),
    last_imu_angular_z_(0.0),
    last_enc_angular_z_(0.0),
    is_first_odom_(true),
    motion_mean_(0.0),
    motion_variance_(4.0),
    measurement_variance_(0.5)
{
    
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 
                                                                           10, 
                                                                           std::bind(&KalmanFilter::odomCallback, 
                                                                                      this, 
                                                                                      std::placeholders::_1)
                                                                        );

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/out",
                                                                        10,
                                                                        std::bind(&KalmanFilter::imuCallback,
                                                                                  this,
                                                                                  std::placeholders::_1)
                                                                    );

    kalman_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman",
                                                                              10
                                                                            );
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    kalman_odometry_ = odom;

    if (is_first_odom_)
    {
        mean_ = odom.twist.twist.angular.z;
        last_enc_angular_z_ = odom.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }
    
    motion_mean_ = odom.twist.twist.angular.z - last_enc_angular_z_;

    statePrediction();
    measurementUpdate();

    last_enc_angular_z_ = odom.twist.twist.angular.z;

    kalman_odometry_.twist.twist.angular.z = mean_;
    kalman_odom_publisher_->publish(kalman_odometry_);
}


void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    last_imu_angular_z_ = imu.angular_velocity.z;
}


void KalmanFilter::measurementUpdate()
{
    mean_ = (measurement_variance_ * mean_ + variance_ * last_imu_angular_z_) / (measurement_variance_ + variance_);
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_);
}

void KalmanFilter::statePrediction()
{
    mean_ = mean_ + motion_mean_;
    variance_ = variance_ + motion_variance_;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_odom_estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}