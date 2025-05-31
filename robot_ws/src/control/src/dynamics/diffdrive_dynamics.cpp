#include "control/include/dynamics/diffdrive_dynamics.hpp"

namespace control::dynamics 
{

using control::state::DiffDriveState;
using control::control_input::DiffDriveControlInput;

std::unique_ptr<control::state::State> DifferentialDriveKinematics::f(
    const control::state::State& state,
    const control::control_input::ControlInput& input) const
{
    const auto& s = static_cast<const DiffDriveState&>(state);
    const auto& u = static_cast<const DiffDriveControlInput&>(input);

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
