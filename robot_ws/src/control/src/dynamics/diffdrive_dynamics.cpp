#include "control/include/dynamics/diffdrive_dynamics.hpp"

namespace control::dynamics
{
    using control::control_input::ControlInput;
    using control::state::State;

    std::unique_ptr<State> DifferentialDriveKinematics::f(
        const State &state,
        const ControlInput &input) const override
    {
        if (state.type() != StateType::DiffDrive || input.type() != ControlInputType::DiffDrive)
        {
            throw std::invalid_argument("Invalid state or control input type for DiffDriveDynamics.");
        }

        const auto &s = static_cast<const DiffDriveState &>(state);
        const auto &u = static_cast<const DiffDriveControlInput &>(input);

        double x = s.x();
        double y = s.y();
        double theta = s.theta();
        double v = u.v();
        double omega = u.omega();

        double x_dot = v * std::cos(theta);
        double y_dot = v * std::sin(theta);
        double theta_dot = omega;

        return std::make_unique<DiffDriveState>(x_dot, y_dot, theta_dot);
    }

} // namespace control::dynamics
