#pragma once
#include "dynamics/dynamics.hpp"
#include "state/ackermann_state.hpp"
#include "control_input/ackermann_control_input.hpp"
#include <cmath>
#include <memory>

namespace control::dynamics
{
    class AckermannDynamics : public Dynamics
    {
    public:

        AckermannDynamics(double wheelbase);
        std::unique_ptr<control::state::State> f(const StateType &state, const control::control_input::ControlInput &input) const override;

    private:
        double wheelbase_;
    };

} // namespace control::dynamics
