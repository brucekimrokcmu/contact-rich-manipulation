#pragma once
#include "control_input/control_input.hpp"
#include "msgs/sensor_data.hpp"

namespace control::behavior
{
    class Behavior
    {
    public:
        virtual ~Behavior() = default;
        virtual void update(double dt, const msgs::SensorData& data) = 0;
        virtual BehaviorMode currentMode() const = 0;

    };

} // control::behavior
