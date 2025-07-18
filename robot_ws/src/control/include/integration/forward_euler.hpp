#pragma once

#include "integration_base.hpp"

namespace control::integration
{
	using control::control_input::ControlInput;
	using control::dynamics::Dynamics;
	using control::state::State;

	class ForwardEuler : public IntegrationBase
	{
	public:
		ForwardEuler(std::unique_ptr<State> x0, const Dynamics &f, double tf, double dt)
			: IntegrationBase(std::move(x0), f, tf, dt)
		{
		}

		void integrate(const ControlInput &u) override
		{
			for (std::size_t i = 1; i < this->num_steps_; ++i)
			{
				State x_next = step(*x_, u);
				*x_ = x_next;
				this->trajectory_.push_back(x_next);
			}
		}

		std::vector<State> integrateTrajectory(const std::vector<ControlInput> &inputs) override
		{
			assert (inputs.size() == this->num_steps_ - 1 && "Inputs size must match number of steps minus one.");
			
			std::vector<State> trajectory(this->num_steps_);
			trajectory[0] = *x0_;

			for (std::size_t i = 1; i < this->num_steps_; ++i)
			{
				State x_next = step(trajectory[i - 1], inputs[i - 1]);
				trajectory[i] = x_next;
			}

			return trajectory;
		}

	private:
		State step(const State &x, const ControlInput &u) const
		{
			

			// Forward Euler method: x_{n+1} = x_n + dt * f(x_n, u)

		}
	};

} // namespace control::integration
