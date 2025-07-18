#pragma once
#include "state/state.hpp"
#include "control_input/control_input.hpp"

namespace control::dynamics
{
	using control::control_input::ControlInput;
	using control::state::State;

	class Dynamics
	{
	public:
		virtual ~Dynamics() = default;
		virtual std::unique_ptr<State> f(const State &state, const ControlInput &input) const = 0;
	};

} // namespace control::dynamics
