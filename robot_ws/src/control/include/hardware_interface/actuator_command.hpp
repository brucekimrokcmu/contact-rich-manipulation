#pragma once

namespace hardware_interface
{
    enum class DriveType : uint8_t
    {
        Differential,
        Ackermann,
        // Extendable
    };

    constexpr size_t MAX_ACTUATORS = 4;

    struct ActuatorCommand
    {
        DriveType type;
        float values[MAX_ACTUATORS];
        uint8_t count;
    };

} // namespace hardware_interface
