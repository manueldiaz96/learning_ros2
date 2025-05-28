#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>

namespace bumperbot_motion
{
PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
    kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), 
    max_angular_velocity_(1.0), 
    previous_angular_error_(0.0), previous_linear_error_(0.0)
{
    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("step_size", step_size_);
    declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    step_size_ = get_parameter("step_size").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));

    last_cycle_time_ = get_clock()->now();

}

bool PDMotionPlanner::transformPlan(const std::string & frame)
{
    // This function transforms plan poses from the 'map' TF to the 'odom' TF
    // this is because the 'odom' TF is the reference for all the control
    // logic as seen in bumperbot_controller/src/simple_controller.cpp

    if(global_plan_.header.frame_id == frame){
        return true;
    }

    geometry_msgs::msg::TransformStamped transform;

    try{
        transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
    } catch (tf2::LookupException & e){
        RCLCPP_ERROR_STREAM(get_logger(), "Could not transform global plan from " << global_plan_.header.frame_id << " to " << frame);
        return false;
    }

    for(auto & pose : global_plan_.poses){
        
        // Syntax is: transform_in, transform_out, transform_to_apply
        tf2::doTransform(pose, pose, transform);
    }

    // Update the general frame for the global_plan object
    global_plan_.header.frame_id = frame;

    return true;
}

void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan_ = *path;
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
    auto next_pose = global_plan_.poses.back();

    for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it){
        double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
        double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > step_size_){
            // Check if the current pose from the path is close enough (step_size) 
            // to the current robot pose
            next_pose = *pose_it;
        } 
        else{
            // In this case, we found the closest pose 
            break;
        }
    }

    return next_pose;

}

void PDMotionPlanner::controlLoop()
{
    if(global_plan_.poses.empty()){
        return;
    }

    geometry_msgs::msg::TransformStamped robot_pose;
    try{
        robot_pose = tf_buffer_->lookupTransform("odom","base_footprint",tf2::TimePointZero);
    }
    catch (tf2::TransformException & e){
        RCLCPP_ERROR(get_logger(), "Not able to transform: %s", e.what());
    }

    bool transform_successful = transformPlan(robot_pose.header.frame_id);

    if(!transform_successful){
        RCLCPP_ERROR_STREAM(get_logger(), "Unable to transform plan from " << global_plan_.header.frame_id << " to " << robot_pose.header.frame_id);
    }

    geometry_msgs::msg::PoseStamped robot_pose_stamped;
    robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

    auto next_pose = getNextPose(robot_pose_stamped);

    double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= 0.1)
    {
        RCLCPP_INFO(get_logger(), "Goal Reached!");
        global_plan_.poses.clear();
        return;
    }

    next_pose_pub_->publish(next_pose);

    tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
    tf2::fromMsg(next_pose.pose, next_pose_tf);

    // Here we are finding the error from our current pose to the next
    // a.k.a. seeing where the next pose is wrt to the local TF of the robot

    next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;

    // Let's calculate the errors (P) and their derivatives (D)

    double d_t = (get_clock()->now() - last_cycle_time_).seconds();

    double linear_error = next_pose_robot_tf.getOrigin().getX();
    double angular_error = next_pose_robot_tf.getOrigin().getY();

    double linear_error_derivative = (linear_error - previous_linear_error_) / d_t;
    double angular_error_derivative = (angular_error - previous_angular_error_) / d_t;

    geometry_msgs::msg::Twist cmd_vel;

    // And now we apply the PD algorithm
    // Keeping in mind to clamp the control effort within the defined bounds

    cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative, -max_linear_velocity_, max_linear_velocity_);
    cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative, -max_angular_velocity_, max_angular_velocity_);

    cmd_pub_->publish(cmd_vel);

    // We update the 'previous_xx' variables for next cycle

    last_cycle_time_ = get_clock()->now();
    previous_linear_error_ = linear_error;
    previous_angular_error_ = angular_error;

}



}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}