#include "behavior/diffdrivefsm_behavior.hpp"
#include "msgs/sensor_data.hpp"

#include <chrono>
#include <iostream>

using namespace control::behavior;
using namespace msgs;

int main()
{
    DiffDriveFSMBehavior fsm;

    constexpr double loop_rate_hz = 10.0;
    constexpr double dt = 1.0/loop_rate_hz;
    constexpr double total_runtime = 5 * 60.0;

    double elapsed = 0.0;

    while (elapsed < total_runtime)
    {
        SensorData sensor;
        sensor.isContact = false;

        fsm.update(dt, sensor);
        elapsed += dt;
    }

    return 0;
}