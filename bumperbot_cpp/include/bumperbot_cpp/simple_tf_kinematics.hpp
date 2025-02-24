#ifndef SIMPLE_TF_KINEMATICS
#define SIMPLE_TF_KINEMATICS 
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bumperbot_interfaces/srv/get_transform.hpp>
#include <bumperbot_interfaces/srv/restart_transform.hpp>


#include <memory>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string &name);

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<bumperbot_interfaces::srv::GetTransform>::SharedPtr get_tf_service_server_;
    rclcpp::Service<bumperbot_interfaces::srv::RestartTransform>::SharedPtr restart_tf_service_server_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};

    rclcpp::TimerBase::SharedPtr timer_;

    double x_increment_;
    double last_x_;
    int rotations_counter_;

    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;


    void timerCallBack();

    bool getTransformCallback(const bumperbot_interfaces::srv::GetTransform::Request::SharedPtr request_,
                              const bumperbot_interfaces::srv::GetTransform::Response::SharedPtr response_);

    bool restartTransformCallback(const bumperbot_interfaces::srv::RestartTransform::Request::SharedPtr request_,
                              const bumperbot_interfaces::srv::RestartTransform::Response::SharedPtr response_);

    
};

#endif