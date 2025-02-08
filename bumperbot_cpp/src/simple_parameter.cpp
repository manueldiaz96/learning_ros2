#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class SimpleParameter : public rclcpp::Node
{
public:
    SimpleParameter() : Node("simple_parameter")
    {
        this->declare_parameter<int>("simple_int_param", 69);
        this->declare_parameter<std::string>("simple_string_param", "Antonio");

        param_Callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::parameterChangeCallback, this, std::placeholders::_1));
    }
private:
    OnSetParametersCallbackHandle::SharedPtr param_Callback_handle_;

    rcl_interfaces::msg::SetParametersResult parameterChangeCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for(const auto& param : parameters)
        {
            if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Parameter simple_int_param changed! New value is: " << param.as_int());
                result.successful = true;
            }
            if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Parameter simple_string_param changed! New value is: " << param.as_string());
                result.successful = true;
            }
        }

        return result;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}