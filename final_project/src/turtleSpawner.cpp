#include "rclcpp/rclcpp.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/remove_turtle.hpp"

#include <random>
#include <chrono>

class TurtleSpawner : public rclcpp::Node
{
public:
    TurtleSpawner() : Node("turtle_spawner")
    {   
        this->declare_parameter<float>("limit_x", 10.0);
        this->declare_parameter<float>("limit_y", 10.0);
        this->declare_parameter<float>("limit_theta", 10.0);
        this->declare_parameter<int>("spawn_rate", 5);

        limit_x = this->get_parameter("limit_x").as_double();
        limit_y = this->get_parameter("limit_y").as_double();
        limit_theta = this->get_parameter("limit_theta").as_double();

        spawn_rate = this->get_parameter("spawn_rate").as_int();

        spawner_ = std::thread(std::bind(&TurtleSpawner::callTurtleSpawn, this));

        pub_turtles_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("/turtles_info", 10);

        remove_turtle_service_ = this->create_service<my_robot_interfaces::srv::RemoveTurtle>("remove_turtle", [this](const my_robot_interfaces::srv::RemoveTurtle::Request::SharedPtr request,
                                                                                                                      my_robot_interfaces::srv::RemoveTurtle::Response::SharedPtr response) {
                                                                                                                      removeTurtleCallback(request, response);});

    }
private:
    std::thread spawner_;
    double limit_x, limit_y, limit_theta;
    int spawn_rate;

    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr pub_turtles_; 
    std::vector<my_robot_interfaces::msg::Turtle> turtles_alive_;

    rclcpp::Service<my_robot_interfaces::srv::RemoveTurtle>::SharedPtr remove_turtle_service_;

    void callTurtleSpawn()
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
    
        // Wait until the server is available
        while (!client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the spawn server to start...");
        }

        while (rclcpp::ok()){
    
            // Create request
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            // Set request parameters
            request->x = generateRandomCoord(0, limit_x);
            request->y = generateRandomCoord(0, limit_y);
            request->theta = generateRandomCoord(0, limit_theta);
        
            // Send async request
            auto future = client->async_send_request(request);
        
            try
            {
                auto response = future.get();
                publishNewTurtle(request->x, request->y, request->theta, response->name);
                
                RCLCPP_INFO_STREAM(this->get_logger(), "Spawn succeeded. Welcome " << response->name);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(spawn_rate));
        }
    }
    
    void publishNewTurtle(float x, float y, float theta, std::string name){
        
        auto turtle = my_robot_interfaces::msg::Turtle();

        turtle.x = x;
        turtle.y = y;
        turtle.theta = theta;
        turtle.name = name;

        turtles_alive_.push_back(turtle);

        auto turtle_array_msg = my_robot_interfaces::msg::TurtleArray();
        turtle_array_msg.turtles = turtles_alive_;

        pub_turtles_->publish(turtle_array_msg);

        RCLCPP_INFO(this->get_logger(), "Updated Turtle List");      


    }

    void removeTurtleCallback(const my_robot_interfaces::srv::RemoveTurtle::Request::SharedPtr request_, 
                                    my_robot_interfaces::srv::RemoveTurtle::Response::SharedPtr response_){
        
        RCLCPP_INFO(this->get_logger(), "Remove Turtle Service Call");      


        std::string turtle_name_ = request_->turtle_name;
        try{
            callKillService(turtle_name_);
            response_->success = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Kill service call failed");
        }

        std::vector<my_robot_interfaces::msg::Turtle> new_turtles_alive_;

        for (size_t i = 0; i < turtles_alive_.size() - 1; i++) {
            new_turtles_alive_.push_back(turtles_alive_[i]);
        }

        turtles_alive_ = new_turtles_alive_;

    }

    void callKillService(std::string turtle_name_){

        auto client = this->create_client<turtlesim::srv::Kill>("kill");
    
        // Wait until the server is available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to start...");
        }
    
        // Create request
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        // Set request parameters
        request->name = turtle_name_;
        // Send async request

        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
            try {
                auto response = future.get();  // Empty response, just ensures service call was successful
                RCLCPP_INFO(this->get_logger(), "Successfully requested to kill turtle: %s", turtle_name_.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out! Turtle %s may not have been removed.", turtle_name_.c_str());
        }

    }

    float generateRandomCoord(float a, float b) {
        // Create a random device and a random number generator
        std::random_device rd;
        std::mt19937 gen(rd()); // Mersenne Twister engine

        // Create a uniform distribution for floating-point numbers between a and b
        std::uniform_real_distribution<> dis(a, b);

        // Generate and return the random number
        return dis(gen);
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}