#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <string>
#include <vector>

#include <libserial/SerialPort.h>


namespace bumperbot_interface{

using callback_return = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class BumperbotInterface: public hardware_interface::SystemInterface
{   
public:
    BumperbotInterface();
    virtual ~BumperbotInterface();

    virtual callback_return on_activate(const rclcpp_lifecycle::State & prev_state) override;
    virtual callback_return on_deactivate(const rclcpp_lifecycle::State & prev_state) override;

    virtual callback_return on_init(const hardware_interface::HardwareInfo & hanrdware_info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
    LibSerial::SerialPort arduino_;

    std::string port_;
    std::vector<double> velocity_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;

    rclcpp::Time last_run_ ;

};

}