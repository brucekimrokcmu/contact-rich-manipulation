#pragma once
#include "control/state/state.hpp"
#include "control/control_input/control_input.hpp"

namespace control::dynamics
{
    class Dynamics
    {
    public:
        virtual ~Dynamics() = default;
        virtual std::unique_ptr<control::state::State> f(const control::state::State &state, const control::contorl_input::ControlInput &input) const = 0;
    };

} // namespace control::dynamics
