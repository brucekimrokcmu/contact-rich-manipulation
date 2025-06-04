#pragma once
#include "behavior/behavior.hpp"

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

class DiffDriveBehavior : public Behavior
{
public:
    DiffDriveBehavior();
    void update(double dt) override;
    bool isCollisionDetected() override;
    BehaviorMode currentMode() const { return mode_; }

private:
    BehaviorMode mode_;
    double timer_;
    void switchMode(BehaviorMode new_mode;)
};

} // namespace control::state_machine
