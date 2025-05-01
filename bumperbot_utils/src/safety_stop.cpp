#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "math.h"

enum State
{
    FREE = 0,
    WARNING = 1,
    DANGER = 2
};

class SafetyStop : public rclcpp::Node
{
public:
    SafetyStop() : Node("safety_stop_node"), is_first_message_(true), state_{State::FREE}, previous_state_{State::FREE} 
    {
        declare_parameter<double>("warning_distance", 0.6);
        declare_parameter<double>("danger_distance", 0.2);

        declare_parameter<std::string>("scan_topic", "scan");
        declare_parameter<std::string>("safety_stop_topic", "safety_stop");

        danger_distance_ = get_parameter("danger_distance").as_double();
        warning_distance_ = get_parameter("warning_distance").as_double();

        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string safety_stop = get_parameter("safety_stop_topic").as_string();

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                                                                      scan_topic, 
                                                                      10,
                                                                      std::bind(&SafetyStop::laserCallback, this, std::placeholders::_1)
                                                                    );

        safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop, 10);

        zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("zones", 10);

        decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");
        increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");

        while (!decrease_speed_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available! Waiting...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        RCLCPP_INFO(get_logger(), "Action /joy_turbo_decrease available!");

        while (!increase_speed_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available! Waiting...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        RCLCPP_INFO(get_logger(), "Action /joy_turbo_increase available!");

        visualization_msgs::msg::Marker warning_zone;
        warning_zone.id = 0;
        warning_zone.action = visualization_msgs::msg::Marker::ADD;
        warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        warning_zone.scale.z = 0.001;
        warning_zone.scale.x = warning_distance_ * 2;
        warning_zone.scale.y = warning_distance_ * 2;
        warning_zone.color.r = 1.0;
        warning_zone.color.g = 0.984;
        warning_zone.color.b = 0.0;
        warning_zone.color.a = 0.5;

        zones_.markers.push_back(warning_zone);

        visualization_msgs::msg::Marker danger_zone;
        danger_zone.id = 1;
        danger_zone.action = visualization_msgs::msg::Marker::ADD;
        danger_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        danger_zone.scale.z = 0.005;
        danger_zone.scale.x = danger_distance_ * 2;
        danger_zone.scale.y = danger_distance_ * 2;
        danger_zone.color.r = 1.0;
        danger_zone.color.g = 0.0;
        danger_zone.color.b = 0.0;
        danger_zone.color.a = 0.5;

        zones_.markers.push_back(danger_zone);

    }
private:
    double danger_distance_, warning_distance_;
    bool is_first_message_;

    State state_, previous_state_;

    visualization_msgs::msg::MarkerArray zones_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

    void laserCallback(const sensor_msgs::msg::LaserScan &msg)
    {
        
        state_ = State::FREE;

        for (const auto & range: msg.ranges)
        {   
            if(!std::isinf(range) && range<= warning_distance_){
                state_ = State::WARNING;

                if (!std::isinf(range) && range<= danger_distance_){
                    state_ = State::DANGER;
                    break;
                }
            }
        }

        if (state_ != previous_state_)
        {
            std_msgs::msg::Bool is_safety_stop;

            if(state_ == State::WARNING){
                is_safety_stop.data = false;
                // Warning marker now solid
                zones_.markers.at(0).color.a = 1.0;
                zones_.markers.at(1).color.a = 0.5;

                decrease_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
                RCLCPP_WARN(get_logger(), "Object within WARNING area, reducing speed...");
            }

            if (state_ == State::DANGER){
                is_safety_stop.data = true;
                // Danger marker now solid 
                zones_.markers.at(1).color.a = 1.0;
                zones_.markers.at(0).color.a = 1.0;

                RCLCPP_WARN(get_logger(), "Object within DANGER area, stopping...");

            }
            else if (state_ == State::FREE){
                is_safety_stop.data = false;
                // When free both markers go back to be transparent
                zones_.markers.at(0).color.a = 0.5;
                zones_.markers.at(1).color.a = 0.5;
                increase_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
                
            }

            safety_stop_pub_->publish(is_safety_stop);
            previous_state_ = state_;
        }

        if(is_first_message_){
            is_first_message_ = false;
            for(auto & zone: zones_.markers){
                zone.header.frame_id = msg.header.frame_id;
            }
        }

        zones_pub_->publish(zones_);

    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}