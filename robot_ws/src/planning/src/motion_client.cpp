#include "planning/motion_client.hpp"

MotionClient::MotionClient()
    : Node("motion_client")
{
    client_ = this->create_client<msgs::srv::PlanMotion>("plan_motion");
}

void MotionClient::sendRequest(double timelimit, bool printPath)
{

    auto request = std::make_shared<msgs::srv::PlanMotion::Request>();
    request->timelimit = timelimit; // TODO: configure in yaml
    request->print_path = printPath;

    while (!client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the motion planning service...");
    }

    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "Planning result: %s", response->path.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionClient>();

    node->sendRequest(5.0, true);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
