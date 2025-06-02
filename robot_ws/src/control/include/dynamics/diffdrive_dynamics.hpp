#pragma once
#include "dynamics/dynamics.hpp"
#include "state/diffdrive_state.hpp"
#include "control_input/diffdrive_control_input.hpp"
#include <memory>

namespace control::dynamics
{
    class DiffDriveDynamics : public Dynamics
    {
    public:
        DiffDriveDynamics() = default;
        std::unique_ptr<control::state::State> f(const StateType &state, const control::control_input::ControlInput &input) const override;
    };

} // namespace control::dynamics
