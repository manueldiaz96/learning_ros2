#include "rclcpp/rclcpp.hpp"
#include "bumperbot_cpp/simple_turtlesim_kinematics.hpp"
#include "turtlesim/msg/pose.hpp"

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &name) : Node(name)
{
    turtle1_pose_sub_ =  this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
            std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, std::placeholders::_1));
    
    turtle2_pose_sub_ =  this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, 
        std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, std::placeholders::_1));
}

void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle1_pose_ = pose;
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle2_pose_ = pose;

    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

    float theta_in_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_in_deg = 180 * theta_in_rad / M_PI;

    RCLCPP_INFO_STREAM(this->get_logger(), "Translation Vector between Turtle1 and Turtle2 \n" << 
        "Tx: " << Tx << " | " <<
        "Ty: " << Ty << "\n" <<
        "Theta in rad: " << theta_in_rad << " | " <<
        "Theta in deg: " << theta_in_deg << "\n" );

    RCLCPP_INFO_STREAM(this->get_logger(), "Rotation Matrix between Turtle1 and Turtle2 \n" << 
        "|R11   R12|" << "\t\t | " << std::cos(theta_in_rad) << "\t" << -std::sin(theta_in_rad) << " |\n" <<
        "|R21   R22|" << "\t\t | " << std::sin(theta_in_rad) << "\t" << std::cos(theta_in_rad) << " |");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}