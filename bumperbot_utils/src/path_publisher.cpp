#include <bumperbot_utils/path_publisher.hpp>

PathPublisher::PathPublisher(const std::string &name) :
    Node(name)
{
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("bumperbot_controller/path", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom",
                                                                    10,
                                                                    std::bind(&PathPublisher::odomCallback, 
                                                                                this, 
                                                                                std::placeholders::_1
                                                                            )
                                                                );

    restart_path_service_ = this->create_service<bumperbot_interfaces::srv::RestartPath>("reset_path",
                std::bind(&PathPublisher::restartPath, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    

}

void PathPublisher::odomCallback(const nav_msgs::msg::Odometry & msg)
{
    last_received_pose_.header.stamp = msg.header.stamp;
    last_received_pose_.header.frame_id = msg.header.frame_id;

    last_received_pose_.pose.position.x = msg.pose.pose.position.x;
    last_received_pose_.pose.position.y = msg.pose.pose.position.y;
    last_received_pose_.pose.position.z = msg.pose.pose.position.z;

    last_received_pose_.pose.orientation.x = msg.pose.pose.orientation.x;
    last_received_pose_.pose.orientation.y = msg.pose.pose.orientation.y;
    last_received_pose_.pose.orientation.z = msg.pose.pose.orientation.z; 
    last_received_pose_.pose.orientation.w = msg.pose.pose.orientation.w;


    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = last_received_pose_.header.frame_id;
    path_msg.poses.push_back(last_received_pose_);

    transform_.header.stamp = path_msg.header.stamp;
    transform_.header.frame_id = path_msg.header.frame_id;
    transform_.child_frame_id = msg.child_frame_id;

    transform_.transform.translation.x = last_received_pose_.pose.position.x;
    transform_.transform.translation.y = last_received_pose_.pose.position.y;
    transform_.transform.translation.z = last_received_pose_.pose.position.z;
    
    transform_.transform.rotation.x = last_received_pose_.pose.orientation.x;
    transform_.transform.rotation.y = last_received_pose_.pose.orientation.y;
    transform_.transform.rotation.z = last_received_pose_.pose.orientation.z;
    transform_.transform.rotation.w = last_received_pose_.pose.orientation.w;

    path_pub_->publish(path_msg);
    tf_broadcaster_->sendTransform(transform_);
}

void PathPublisher::restartPath(const bumperbot_interfaces::srv::RestartPath::Request::SharedPtr & request,
                                const bumperbot_interfaces::srv::RestartPath::Response::SharedPtr & response)
{
    try {
        if (request->restart)
        {
            path_msg.poses.clear();
            RCLCPP_INFO(this->get_logger(), "Path reset successfully.");
            response->success = true;  // Indicate that the path was reset successfully
        }
        else
        {
            response->success = false;  // If the restart flag is false, indicate failure
            RCLCPP_WARN(this->get_logger(), "Path reset request was not triggered.");
        }
    }
    catch (const std::exception &e) 
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "An error occured while resetting the path. ");
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
    }
    
    response->success = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>("bumperbot_path_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}