#pragma once
#include "state/state.hpp"
#include "control_input/control_input.hpp"

namespace control::dynamics
{
    class Dynamics
    {
    public:
        virtual ~Dynamics() = default;
        virtual std::unique_ptr<control::state::State> f(const control::state::State &state, const control::control_input::ControlInput &input) const = 0;
    };

} // namespace control::dynamics
