#pragma once
#include "control/control_input/control_input.hpp"

namespace control::behavior
{
    class Behavior
    {
    public:
        virtual ~Behavior() = default;
        virtual void update(double dt) = 0;
        virtual void onCollisionDetected() = 0;
    };

} // control::behavior
