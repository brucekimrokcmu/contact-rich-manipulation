#include "behavior/diffdrive_behavior.hpp"
#include <cstdlib>
#include <ctime>

namespace control::behavior
{

    DiffDriveBehavior::DiffDriveBehavior()
        : mode_(BehaviorMode::IDLE),
          timer_(0.0)
    {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        switchMode(BehaviorMode::FORWARD);
    }

    void DiffDriveBehavior::update(double dt)
    {
        // TODO
    }

    bool DiffDriveBehavior::isCollisionDetected() override
    {
        // TODO: interface with sensor
        return true;
    }

    void DiffDriveBehavior::switchMode(BehaviorMode new_mode)
    {
        if (mode_ == new_mode)
        {
            return;
        }void
        mode_ = new_mode;

        switch (mode_)
        {
        // TODO: Add effects of mode switch. i.e. timer_ = t;
        case BehaviorMode::FORWARD:
            break;
        case BehaviorMode::REVERSE:
            break;
        case BehaviorMode::TURNCW:
            break;
        case BehaviorMode::TURNCCW:
            break;
        case BehaviorMode::IDLE:
            timer_ = 0.0;
            break;
        default:
            timer_ = 0.0;
            break;
        }    
    }

    double DiffDriveBehavior::randomDuration(double min, double max) const
    {
        double r = static_cast<double>(std::rand()) / RAND_MAX;
        return min + r * (max - min);
    }

} // namespace control::behavior
