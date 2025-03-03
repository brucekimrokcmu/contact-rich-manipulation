#include "planning/motion_server.hpp"

MotionServer::MotionServer()
    : Node("simple_motion_plannernode")
{
    service_ = this->create_service<msgs::srv::PlanMotion>(
        "plan_motion",
        std::bind(&MotionServer::handlePlanMotionRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MotionServer is ready.");
}

void MotionServer::handlePlanMotionRequest(
    const std::shared_ptr<msgs::srv::PlanMotion::Request> request,
    std::shared_ptr<msgs::srv::PlanMotion::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received a motion plan request");

    bool success = planner.planRRTstar(request->timelimit, request->print_path);
    response->success = success;
    response->path = success ? "Planned path generated successfully." : "Planning failed.";
    RCLCPP_INFO(this->get_logger(), "Planning %s", success ? "succeeded" : "failed");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}