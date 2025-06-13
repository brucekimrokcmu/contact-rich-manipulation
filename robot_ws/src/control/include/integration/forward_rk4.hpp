#pragma once

#include "integration_base.hpp"

namespace control::integration
{
	using control::control_input::ControlInput;
	using control::dynamics::Dynamics;
	using control::state::State;

	class ForwardRK4 : public IntegratorBase
	{
	public:
		ForwardRK4(std::unique_ptr<State> &x0, const Dynamics &f, const ControlInput &u, double tf, double dt)
			: IntegrationBase(std::move(x0), f, tf, dt)
		{
		}

		void integrate(const ControlInput &u) override
		{
			for (std::size_t i = 1; i < this->num_steps_; ++i)
			{

				step(*x_, u)
			}
		}

		std::vector<State> integrateTrajectory(const std::vector<ControlInput> &inputs) override
		{
			assert(inputs.size() == this->num_steps_ - 1 && "Inputs size must match number of steps minus one.");
			std::vector<State> trajectory(this->num_steps_);
			trajectory[0] = this->x0_;

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
			const double dt = this->dt_;
			const auto &f = this->f_;

			// Runge-Kutta 4th order method
			State k1 = f(x, u);
			State k2 = f(x + 0.5 * dt * k1, u);
			State k3 = f(x + 0.5 * dt * k2, u);
			State k4 = f(x + dt * k3, u);

			return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
		}
	};

} // namespace control::integration
