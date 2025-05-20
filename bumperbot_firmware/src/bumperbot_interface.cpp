#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace bumperbot_interface{

BumperbotInterface::BumperbotInterface(){

}

BumperbotInterface::~BumperbotInterface(){
    if(arduino_.IsOpen()){
        try{
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), "Something is wrong with port:=" << port_);
        }
    }
}
callback_return BumperbotInterface::on_init(const hardware_interface::HardwareInfo & hardwareinfo){

    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardwareinfo);

    if (result != CallbackReturn::SUCCESS){
            return result;
    }
    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "No Serial port is provided! ABORTING!!");
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces(){
    
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(size_t i=0; i < info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_POSITION, &position_states_.at(i))); 

        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_states_.at(i))); 
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces(){
    
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i=0; i < info_.joints.size(); i++){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_commands_.at(i))); 

    }

    return command_interfaces;
}

callback_return BumperbotInterface::on_activate(const rclcpp_lifecycle::State & prev_state){
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware!!");

    velocity_commands_ = {0.0, 0.0, 0.0, 0.0};
    position_states_ = {0.0, 0.0, 0.0, 0.0};
    velocity_states_ = {0.0, 0.0, 0.0, 0.0};

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); 
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), "Something went wrong when interacting with port:=" << port_);
        return CallbackReturn::FAILURE;

    }
    
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware connected successfully!!, READY TO INITIALLIZE");



}
}