#pragma once

#include "hardware_interface/motor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace hardware_interface
{
    class MicroROSMotorInterface : public MotorInterface
    {
    public:
        explicit MicroROSMotorInterface(rclcpp::Node *node)
            : node_(node)
        {
            publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_cmd", rclcpp::QoS(1).best_effort());
        }

        void sendCommand(const ActuatorCommand &cmd) override
        {
            if (cmd.type != DriveType::Differential || cmd.count != 2)
            {
                RCLCPP_ERROR(logger_, "Invalid ActuatorCommand: expected Differential with 2 values, got type=%d, count=%zu", static_cast<int>(cmd.type), cmd.count);
                return;
            }

            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(2);
            msg.data[0] = cmd.values[0];
            msg.data[1] = cmd.values[1];
            publisher_->publish(msg);
        }

        DriveType driveType() const override
        {
            return DriveType::Differential;
        }

    private:
        rclcpp::Node *node_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Logger logger_ = node_->get_logger();
    };

} // namespace hardware_interface
