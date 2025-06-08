#pragma once
#include "hardware_interface/actuator_command.hpp"

namespace hardware_interface
{

    class MotorInterface
    {
    public:
        virtual ~MotorInterface() = default;
        virtual void sendCommand(const ActuatorCommand &cmd) = 0;
        virtual DriveType driveType() const = 0;
    };

} // namespace hardware_interface
