#include <bumperbot_cpp/simple_tf_kinematics.hpp>

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : 
    Node(name), 
    x_increment_(0.005), 
    last_x_(0.0),
    rotations_counter_(0.0)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    static_transform_stamped_.header.stamp = this->get_clock()->now();

    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";

    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.03;

    // With this quaternion we say that the two TFs have the same orientation

    static_transform_stamped_.transform.rotation.x = 0.0 ;
    static_transform_stamped_.transform.rotation.y = 0.0 ;
    static_transform_stamped_.transform.rotation.z = 0.0 ;
    static_transform_stamped_.transform.rotation.w = 1.0 ;

    static_tf_broadcaster_->sendTransform(static_transform_stamped_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing static transform between " << static_transform_stamped_.header.frame_id << 
                                            " to " << static_transform_stamped_.child_frame_id);

    last_orientation_.setRPY(0.0, 0.0, 0.0) ;
    orientation_increment_.setRPY(0.0, 0.0, 0.05);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimpleTfKinematics::timerCallBack, this));

    get_tf_service_server_ = this->create_service<bumperbot_interfaces::srv::GetTransform>("get_transform", 
                               std::bind(&SimpleTfKinematics::getTransformCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    restart_tf_service_server_ = this->create_service<bumperbot_interfaces::srv::RestartTransform>("restart_transform", 
                               std::bind(&SimpleTfKinematics::restartTransformCallback, this, std::placeholders::_1, std::placeholders::_2));
    
};

void SimpleTfKinematics::timerCallBack(){

    dynamic_transform_stamped_.header.stamp = this->get_clock()->now();

    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q = last_orientation_ * orientation_increment_;
    q.normalize();

    dynamic_transform_stamped_.transform.rotation.x = q.x();
    dynamic_transform_stamped_.transform.rotation.y = q.y();
    dynamic_transform_stamped_.transform.rotation.z = q.z();
    dynamic_transform_stamped_.transform.rotation.w = q.w();
    
    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);

    last_x_ = dynamic_transform_stamped_.transform.translation.x;

    rotations_counter_++;

    last_orientation_ = tf2::Quaternion(dynamic_transform_stamped_.transform.rotation.x,
                                        dynamic_transform_stamped_.transform.rotation.y,
                                        dynamic_transform_stamped_.transform.rotation.z,
                                        dynamic_transform_stamped_.transform.rotation.w);

    if (rotations_counter_ >= 100){
        orientation_increment_ = orientation_increment_.inverse();
        rotations_counter_ = 0;
        x_increment_ *= -1;
    }
    

    RCLCPP_INFO_STREAM(this->get_logger(), "Number of rotations: " << rotations_counter_);

}

bool SimpleTfKinematics::getTransformCallback(const bumperbot_interfaces::srv::GetTransform::Request::SharedPtr request_,
                                                    bumperbot_interfaces::srv::GetTransform::Response::SharedPtr response_)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Requested Transform between " << request_->frame_id << " and " << request_->child_frame_id);

    geometry_msgs::msg::TransformStamped requested_transform;

    try
    {
        requested_transform = tf_buffer_->lookupTransform(request_->frame_id,
                                                          request_->child_frame_id,
                                                          tf2::TimePointZero); // This means get the latest transform
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "An error occurred when transforming from " << request_->frame_id << " to " << request_->child_frame_id);
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());

        response_->success = false;
        return true;
    }
    
    response_->transform = requested_transform;

    response_->success = true;
    return true;
}

bool SimpleTfKinematics::restartTransformCallback(const bumperbot_interfaces::srv::RestartTransform::Request::SharedPtr request_,
                                             const bumperbot_interfaces::srv::RestartTransform::Response::SharedPtr response_)
{
    try {
        dynamic_transform_stamped_.header.stamp = this->get_clock()->now();

        dynamic_transform_stamped_.header.frame_id = "odom";
        dynamic_transform_stamped_.child_frame_id = "bumperbot_base";

        dynamic_transform_stamped_.transform.translation.x = 0.0;
        dynamic_transform_stamped_.transform.translation.y = 0.0;
        dynamic_transform_stamped_.transform.translation.z = 0.0;

        dynamic_transform_stamped_.transform.rotation.x = 0.0;
        dynamic_transform_stamped_.transform.rotation.y = 0.0;
        dynamic_transform_stamped_.transform.rotation.z = 0.0;
        dynamic_transform_stamped_.transform.rotation.w = 1.0;
        
        dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);

        last_x_ = dynamic_transform_stamped_.transform.translation.x;

        last_orientation_ = tf2::Quaternion(dynamic_transform_stamped_.transform.rotation.x,
                                            dynamic_transform_stamped_.transform.rotation.y,
                                            dynamic_transform_stamped_.transform.rotation.z,
                                            dynamic_transform_stamped_.transform.rotation.w);

        rotations_counter_ = 0;
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "An error occurred when transforming from " << request_->frame_id << " to " << request_->child_frame_id);
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());

        response_->success = false;
        return true;
    }

    response_->success = true;
    return true;
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}