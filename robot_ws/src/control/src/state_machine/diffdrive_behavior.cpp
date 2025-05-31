#include "control/include/state_machine/diffdrive_behavior.hpp"

namespace control::behavior
{
    DiffDriveBehavior::DiffDriveBehavior()
        :  mode_(BehaviorMode::IDLE)

    // TODO: define any behavior change when mode switches
    void DiffDriveBehavior::switchMode(BehaviorMode new_mode)
    {
        mode_ = new_mode;

        swich(mode_)
        {
            case BehaviorMode::IDLE:
                break;
            case BehaviorMode::FORWARD:
                break;
            case BehaviorMode::REVERSE:
                break;
            case BehaviorMode::TURNCW:
                break;                
            case BehaviorMode::TURNCCW:
                break;
            default:
                break;
        }

    }

} // namespace control::behavior

