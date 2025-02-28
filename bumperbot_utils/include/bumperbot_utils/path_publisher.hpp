#ifndef PATH_PUBLISHER
#define PATH_PUBLISHER

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include <bumperbot_interfaces/srv/restart_path.hpp>


class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher(const std::string &name);

private:
    std::string frame_id_;

    geometry_msgs::msg::PoseStamped last_received_pose_;
    nav_msgs::msg::Path path_msg;

    geometry_msgs::msg::TransformStamped transform_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Service<bumperbot_interfaces::srv::RestartPath>::SharedPtr restart_path_service_;

    void odomCallback(const nav_msgs::msg::Odometry & msg);

    void restartPath(const bumperbot_interfaces::srv::RestartPath::Request::SharedPtr & request,
                     const bumperbot_interfaces::srv::RestartPath::Response::SharedPtr & response);

};

#endif