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
    turtleController() : Node("turtle_controller"), turtlesim_up_(false), name_("turtle1"), target_name_("_")
    {
        target_ = {5.4 , 5.4};
        subscriber_pose_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&turtleController::callbackPose, this, std::placeholders::_1));
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&turtleController::controlLoop, this));

        subscriber_turtle_array = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("turtles_info", 10, std::bind(&turtleController::updateTurtleList, this, std::placeholders::_1));
        // publisher_last_turtle_ = this->create_publisher<std_msgs::msg::String>("remove_turtle", 10);

        RCLCPP_INFO(this->get_logger(), "Started turtle controller");
    }
private:
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;
    std::string name_, target_name_;

    target_struct target_;
    std::vector<my_robot_interfaces::msg::Turtle> turtle_targets_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr subscriber_turtle_array;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_last_turtle_;

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
        float goal_theta =  std::atan2(dist_y, dist_x);

        // RCLCPP_INFO_STREAM(this->get_logger(), "Distance: " << distance);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Goal Theta: " << goal_theta);

        auto cmd_vel_msg = geometry_msgs::msg::Twist();

        if(distance > 0.5)
        {
            
            float angle_diff = goal_theta - pose_.theta;
            
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if(angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            
            cmd_vel_msg.linear.x = 2*distance;
            cmd_vel_msg.angular.z =  6*angle_diff; 
        }
        else{
            // RCLCPP_INFO(this->get_logger(), "Already at position");
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;
            updateTarget();
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

    void updateTarget(){
        
        if (!turtle_targets_.empty()){

            RCLCPP_INFO(this->get_logger(), "Updating Target");      

            if (target_name_ != "_"){
                callRemoveTurtle(target_name_);
                

                std::string removeTarget = target_name_;
                turtle_targets_.erase(
                    std::remove_if(turtle_targets_.begin(), turtle_targets_.end(),
                        [&removeTarget](my_robot_interfaces::msg::Turtle& t) {
                            return t.name == removeTarget;
                        }),
                    turtle_targets_.end()
                    );
            }
        }
        if (!turtle_targets_.empty()){
            // Using back()
            auto& last_turtle = turtle_targets_.back();
            
            // OR using size-1 as index
            // const auto& last_turtle2 = turtle_targets_[turtle_targets_.size() - 1];

            // Now you can access the last turtle's attributes
            std::string name = last_turtle.name;
            target_.x = last_turtle.x;
            target_.y = last_turtle.y;
            target_name_ = last_turtle.name;
            RCLCPP_INFO_STREAM(this->get_logger(), "New target: " << target_name_ );
        }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Turtle queue empty!");
        // }

    }
    void callRemoveTurtle(std::string turtle_name){

        RCLCPP_INFO_STREAM(this->get_logger(), "Call to remove " << turtle_name);      


        auto client = this->create_client<my_robot_interfaces::srv::RemoveTurtle>("remove_turtle");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to start...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::RemoveTurtle::Request>();
        // Set request parameters
        request->turtle_name = turtle_name;

        // Send async request
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO_STREAM(this->get_logger(), "Service Call Succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
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