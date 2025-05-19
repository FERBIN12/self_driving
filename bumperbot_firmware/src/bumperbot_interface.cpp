#include "bumperbot_firmware/bumperbot_interface.hpp"

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

}