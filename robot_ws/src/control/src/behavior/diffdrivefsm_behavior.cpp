#include "behavior/diffdrivefsm_behavior.hpp"
#include <cstdlib>
#include <ctime>

namespace control::behavior
{

    DiffDriveFSMBehavior::DiffDriveFSMBehavior()
        : mode_(BehaviorMode::IDLE),
          mode_timer_(5.0) // Start in IDLE for 5 sec
    {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
    }

    void DiffDriveFSMBehavior::update(double dt, const msgs::SensorData &data) override
    {
        // TODO
        mode_timer_ -= dt;
        bool isContact = data.isContact;

        switch (mode_)
        {
        case BehaviorMode::FORWARD:
            if (isContact)
            {
                switchMode(BehaviorMode::REVERSE);
                mode_timer_ = randomDuration(1.0, 2.0);
            }   
            break;

        case BehaviorMode::REVERSE:
            if (mode_timer_ <= 0.0)
            {
                if (std::rand() % 2 == 0)
                {
                    switchMode(BehaviorMode::TURNCW);
                }
                else
                {
                    switchMode(BehaviorMode::TURNCCW);
                }
                mode_timer_ = randomDuration(1.0, 2.0); // TODO: calculate time amount for 90 degrees turn
            }
            break;
        
        case BehaviorMode::TURNCW:
        case BehaviorMode::TURNCCW:
            if (mode_timer_ <= 0.0)
            {
                switchMode(BehaviorMode::FORWARD);
                mode_timer_ = 0.0;
            }
            break;
        
        case BehaviorMode::IDLE:
            if (mode_timer_ <= 0.0)
            {
                switchMode(BehaviorMode::FORWARD);
            }
            break;

        default:
            break;
        }
    }

    bool DiffDriveFSMBehavior::isContact()
    {
        return false;
    }

    void DiffDriveFSMBehavior::switchMode(BehaviorMode new_mode)
    {
        if (mode_ == new_mode)
        {
            return;
        }

        mode_ = new_mode;

        switch (mode_)
        {
        // TODO: Add effects of mode switch. i.e. mode_timer_ = t;
        case BehaviorMode::FORWARD:
            mode_timer_ = 0.0;        
            break;

        case BehaviorMode::REVERSE:
        case BehaviorMode::TURNCW:
        case BehaviorMode::TURNCCW:
            break;

        case BehaviorMode::IDLE:
        default:
            mode_timer_ = 5.0;
            break;
        }
    }

    double DiffDriveFSMBehavior::randomDuration(double min, double max) const
    {
        double r = static_cast<double>(std::rand()) / RAND_MAX;
        return min + r * (max - min);
    }

} // namespace control::behavior
