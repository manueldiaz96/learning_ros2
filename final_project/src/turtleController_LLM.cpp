#include "rclcpp/rclcpp.hpp"

#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/srv/remove_turtle.hpp"

struct target_struct {
    float x;
    float y;
};

class turtleController : public rclcpp::Node
{
public:
    turtleController() : Node("turtle_controller"), 
                        turtlesim_up_(false), 
                        name_("turtle1"), 
                        target_name_("_"),
                        is_removing_turtle_(false)
    {
        target_ = {5.4 , 5.4};
        subscriber_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, 
            std::bind(&turtleController::callbackPose, this, std::placeholders::_1));
            
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10);
        
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&turtleController::controlLoop, this));

        subscriber_turtle_array = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "turtles_info", 10, 
            std::bind(&turtleController::updateTurtleList, this, std::placeholders::_1));

        remove_client_ = this->create_client<my_robot_interfaces::srv::RemoveTurtle>("remove_turtle");

        RCLCPP_INFO(this->get_logger(), "Started turtle controller");
    }

private:
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;
    bool is_removing_turtle_;
    std::string name_, target_name_;
    target_struct target_;
    std::vector<my_robot_interfaces::msg::Turtle> turtle_targets_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr subscriber_turtle_array;
    rclcpp::Client<my_robot_interfaces::srv::RemoveTurtle>::SharedPtr remove_client_;

    void callbackPose(turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = *msg.get();
        turtlesim_up_ = true;
    }

    void controlLoop()
    {
        if (!turtlesim_up_)
        {  
           RCLCPP_INFO(this->get_logger(), "Waiting for turtle to appear");
           return;
        }

        float dist_x = target_.x - pose_.x;
        float dist_y = target_.y - pose_.y;
        float distance = std::sqrt(dist_x*dist_x + dist_y*dist_y);
        float goal_theta = std::atan2(dist_y, dist_x);

        auto cmd_vel_msg = geometry_msgs::msg::Twist();

        if(distance > 0.5)
        {
            float angle_diff = goal_theta - pose_.theta;
            
            if (angle_diff > M_PI)
                angle_diff -= 2 * M_PI;
            else if(angle_diff < -M_PI)
                angle_diff += 2 * M_PI;
            
            cmd_vel_msg.linear.x = 2*distance;
            cmd_vel_msg.angular.z = 6*angle_diff; 
        }
        else {
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;
            
            // Only update target if we're not currently removing a turtle
            if (!is_removing_turtle_) {
                updateTarget();
            }
        }

        publisher_cmd_vel_->publish(cmd_vel_msg);
    }

    void updateTurtleList(my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty()) {
            turtle_targets_ = msg->turtles;  
            RCLCPP_INFO(this->get_logger(), "Updated Turtle List");      
        }
    }

    void updateTarget()
    {
        if (turtle_targets_.empty()) {
            return;
        }

        if (target_name_ != "_") {
            is_removing_turtle_ = true;
            auto request = std::make_shared<my_robot_interfaces::srv::RemoveTurtle::Request>();
            request->turtle_name = target_name_;

            // Send async request with callback
            auto remove_callback = [this](rclcpp::Client<my_robot_interfaces::srv::RemoveTurtle>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result->success) {
                        RCLCPP_INFO_STREAM(this->get_logger(), "Successfully removed turtle: " << target_name_);
                        // Remove from our list
                        turtle_targets_.erase(
                            std::remove_if(turtle_targets_.begin(), turtle_targets_.end(),
                                [this](const auto& t) { return t.name == target_name_; }),
                            turtle_targets_.end()
                        );
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to remove turtle: %s", e.what());
                }
                is_removing_turtle_ = false;
                
                // Select next target immediately after removal
                selectNextTarget();
            };

            if (!remove_client_->wait_for_service(std::chrono::milliseconds(100))) {
                RCLCPP_WARN(this->get_logger(), "Remove service not available");
                is_removing_turtle_ = false;
                return;
            }

            remove_client_->async_send_request(request, remove_callback);
        } else {
            selectNextTarget();
        }
    }

    void selectNextTarget() {
        if (!turtle_targets_.empty()) {
            const auto& next_turtle = turtle_targets_.back();
            target_.x = next_turtle.x;
            target_.y = next_turtle.y;
            target_name_ = next_turtle.name;
            RCLCPP_INFO_STREAM(this->get_logger(), "New target: " << target_name_);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}