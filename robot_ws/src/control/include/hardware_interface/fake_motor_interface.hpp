#pragma once
#include "motor_interface.hpp"
#include <cstdio>

namespace hardware_interface
{

    class FakeMotorInterface : public MotorInterface
    {
    public:
        void sendCommand(const ActuatorCommand &cmd) override
        {
            printf("[FAKE] Motor command: ");
            for (uint8_t i = 0; i < cmd.count; ++i)
            {
                printf("%.2f ", cmd.values[i]);
            }
            printf("\n");
        }

        DriveType driveType() const override
        {
            return DriveType::Differential;
        }
    };

} // namespace hardware_interface
