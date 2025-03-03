#pragma once
#include "rclcpp/rclcpp.hpp"
#include <msgs/srv/plan_motion.hpp>
#include <chrono>

class MotionClient : public rclcpp::Node
{
public:
    MotionClient();
    void sendRequest(double timelimit, bool printPath);
private:
    rclcpp::Client<msgs::srv::PlanMotion>::SharedPtr client_;
};