#pragma once
#include "behavior/behavior.hpp"
#include "msgs/sensor_data.hpp"

namespace control::behavior
{

enum struct BehaviorMode
{
    IDLE,
    FORWARD,
    REVERSE,
    TURNCW,
    TURNCCW
};

class DiffDriveFSMBehavior : public Behavior
{
public:
    DiffDriveFSMBehavior();
    void update(double dt, const msgs::SensorData& data) override;
    BehaviorMode currentMode() const override { return mode_; } 

private:
    BehaviorMode mode_;
    double mode_timer_;
    void switchMode(BehaviorMode new_mode);
    double randomDuration(double min, double max) const;
};

} // namespace control::behavior
