#pragma once
#include "rclcpp/rclcpp.hpp"
#include "planning/simple_motion_planner.hpp"
#include "msgs/srv/plan_motion.hpp" // generated from projects/Fastening/robot_ws/src/planning/msg/plan_motion.srv

class MotionServer : public rclcpp::Node
{
public:
    MotionServer();

private:
    void handlePlanMotionRequest(const std::shared_ptr<msgs::srv::PlanMotion::Request> request,
                                 std::shared_ptr<msgs::srv::PlanMotion::Response> response);

    rclcpp::Service<msgs::srv::PlanMotion>::SharedPtr service_;
    SimpleMotionPlanner planner;
};
