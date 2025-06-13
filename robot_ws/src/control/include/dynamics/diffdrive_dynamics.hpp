#pragma once
#include "dynamics/dynamics.hpp"
#include "state/diffdrive_state.hpp"
#include "control_input/diffdrive_control_input.hpp"
#include <memory>

namespace control::dynamics
{
	using control::control_input::ControlInput;
	using control::state::State;

	class DiffDriveDynamics : public Dynamics
	{
	public:
		DiffDriveDynamics() = default;
		std::unique_ptr<State> f(const State &state, const ControlInput &input) const override;
	};

} // namespace control::dynamics
