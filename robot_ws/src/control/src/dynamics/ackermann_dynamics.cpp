#include "control/include/dynamics/ackermann_dynamics.hpp"

namespace control::dynamics
{
    using control::control_input::ControlInput;
    using control::state::State;

    AckermannDynamics::AckermannDynamics(double wheelbase)
        : wheelbase_(wheelbase)
    {
    }

    std::unique_ptr<State> AckermannDynamics::f(const State &state, const ControlInput &input) const override
    {
        if (state.type() != StateType::Ackermann || input.type() != ControlInputType::Ackermann)
        {
            throw std::invalid_argument("Invalid state or control input type for AckermannDynamics.");
        }

        const auto &s = static_cast<const AckermannState &>(state);
        const auto &u = static_cast<const AckermannControlInput &>(input);

        double x = s.x();
        double y = s.y();
        double theta = s.theta();
        double delta = s.delta();
        double v = s.v();
        double L = wheelbase_;
        // TODO: Assume center as a desired point
        double x_dot = v * std::cos(delta + theta);
        double y_dot = v * std::sin(delta + theta);
        double theta_dot = v * std::sin(delta) / L;
        double delta_dot = u.phi();
        double v_dot = u.a();

        return std::make_unique<AckermannState>(x_dot, y_dot, theta_dot, delta_dot);
    }

} // namespace control::dynamics
