#include "bumperbot_motion/pure_pursuit.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace bumperbot_motion
{
PurePursuit::PurePursuit() : Node("pure_pursuit_motion_planner_node"),
    look_ahead_distance_(0.5) , max_linear_velocity_(0.3), max_angular_velocity_(1.0)
{
    declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
    declare_parameter<double>("look_ahead_distance", look_ahead_distance_);

    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    carrot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));

    last_cycle_time_ = get_clock()->now();

}

bool PurePursuit::transformPlan(const std::string & frame)
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

void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan_ = *path;
}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
    auto carrot_pose = global_plan_.poses.back();

    for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it){
        double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
        double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > look_ahead_distance_){
            // Check if the carrot pose from the path is close enough (look_ahead_distance_) 
            // to the current robot pose
            carrot_pose = *pose_it;
        } 
        else{
            // In this case, we found the closest pose 
            break;
        }
    }

    return carrot_pose;

}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose){

    const double L = (carrot_pose.position.x * carrot_pose.position.x) + 
                     (carrot_pose.position.y * carrot_pose.position.y);
    if (L > 0.001)
    {
        return 2.0 * carrot_pose.position.y / L;
    }
    else{
        return 0.0;
    }
}


void PurePursuit::controlLoop()
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

    auto carrot_pose = getCarrotPose(robot_pose_stamped);
    carrot_pose_pub_->publish(carrot_pose);

    double dx = carrot_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy = carrot_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= 0.1)
    {
        RCLCPP_INFO(get_logger(), "Goal Reached!");
        global_plan_.poses.clear();
        return;
    }

    tf2::Transform robot_tf, carrot_pose_tf, carrot_pose_robot_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
    tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);

    // Here we are finding the error from our current pose to the next
    // a.k.a. seeing where the next pose is wrt to the local TF of the robot

    carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;
    tf2::toMsg(carrot_pose_tf, carrot_pose.pose);

    double curvature = getCurvature(carrot_pose.pose);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_linear_velocity_;
    cmd_vel.angular.z = curvature * max_angular_velocity_; 
    cmd_pub_->publish(cmd_vel);
}



}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}