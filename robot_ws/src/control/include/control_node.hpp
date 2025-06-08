#pragma once

#include "behavior/diffdrivefsm_behavior.hpp"
#include "control_input/diffdrive_control_input.hpp"
#include "hardware_interface/micro_ros_motor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/sensor_data.hpp" 

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();

private:
    void controlLoop();
    rclcpp::Subscription<msgs::msg::SensorData>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    control::behavior::DiffDriveFSMBehavior behavior_;
    

};