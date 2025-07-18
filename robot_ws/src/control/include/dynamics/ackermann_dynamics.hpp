#pragma once
#include "dynamics/dynamics.hpp"
#include "state/ackermann_state.hpp"
#include "control_input/ackermann_control_input.hpp"
#include <cmath>
#include <memory>

namespace control::dynamics
{
	using control::control_input::ControlInput;
	using control::state::State;

	class AckermannDynamics : public Dynamics
	{
	public:
		AckermannDynamics(double wheelbase);
		std::unique_ptr<State> f(const State &state, const ControlInput &input) const override;

	private:
		double wheelbase_;
	};

} // namespace control::dynamics
