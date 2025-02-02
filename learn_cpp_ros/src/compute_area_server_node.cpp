#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ComputeAreaServer : public rclcpp::Node
{
public:
    ComputeAreaServer() : Node("compute_area_server")
    {
        server_ = this->create_service<my_robot_interfaces::srv::ComputeRectangleArea>(
            "computer_rectangle_area",
            std::bind(&ComputeAreaServer::callbackComputeRectangleArea, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Compute Area Server Online");
    }
private:
    rclcpp::Service<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr server_;

    void callbackComputeRectangleArea(const my_robot_interfaces::srv::ComputeRectangleArea::Request::SharedPtr request_,
                                      const my_robot_interfaces::srv::ComputeRectangleArea::Response::SharedPtr response_)
    {
        response_->area = request_->height * request_->width;
        RCLCPP_INFO(this->get_logger(), "Base %f | Width: %f | Area: %f", request_->height, request_->width, response_->area);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeAreaServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}