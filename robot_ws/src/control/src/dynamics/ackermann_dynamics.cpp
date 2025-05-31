#include "control/include/dynamics/ackermann_dynamics.hpp"

namespace control::dynamics
{
    using namespace control::state::AckermannState;
    using namespace control::control_input::AckermannControlInput;

    AckermannDynamics::AckermannDynamics(double wheelbase)
        : wheelbase_(wheelbase)
    {
    }

    std::unique_ptr<control::state::State> AckermannDynamics::f(const StateType &state, const control::control_input::ControlInput &input) const override
    {
        const auto &s = static_cast<const AckermannState&>(state);
        const auto &u = static_cast<const AckermannControlInput&>(input);

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
