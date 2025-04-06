#include "bumperbot_arduino/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace bumperbot_arduino
{
    BumperbotInterface::BumperbotInterface()
    {

    }
    BumperbotInterface::~BumperbotInterface()
    {
        try{
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), "Something went wrong while closing the connection with the port " << port_);
        }
    }

    CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if(result != CallbackReturn::SUCCESS){
            return result;
        }

        try{
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "No serial port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION));
        }
    }

}