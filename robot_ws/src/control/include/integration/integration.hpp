#pragma once
#include <cmath>
#include <vector>
#include <memory>
#include "state/state.hpp"
#include "control_input/control_input.hpp"
#include "dynamics/dynamics.hpp"

namespace control::integration
{
	using control::control_input::ControlInput;
	using control::dynamics::Dynamics;
	using control::state::State;

	class IntegrationBase
	{
	public:
		// TODO: Reconsider the use of unique_ptr for x0_ if it is not modified after initialization
		IntegrationBase(std::unique_ptr<State> x0, const Dynamics &f, double tf, double dt)
			: x_(std::move(x0)),
			  x0_(x_->clone()),
			  f_(f),
			  tf_(tf),
			  dt_(dt),
			  num_steps_(static_cast<std::size_t>(std::round(tf / dt_)) + 1)
		{

			trajectory_.reserve(num_steps_);
			trajectory_.push_back(*x0_);
			time_vec_.clear();
			time_vec_.resize(num_steps_);

			time_vec_.reserve(num_steps_);
			for (std::size_t i = 0; i < num_steps_; ++i)
			{
				time_vec_.push_back(i * dt_);
			}


		}

		virtual ~IntegrationBase() = default;
		virtual void integrate(const ControlInput &u) = 0;
		virtual std::vector<State> integrateTrajectory(const std::vector<ControlInput> &inputs) = 0;

	protected:
		std::unique_ptr<State> x_;
		std::unique_ptr<State> x0_;
		const Dynamics &f_;
		double tf_;
		double dt_;
		std::size_t num_steps_;
		std::vector<State> trajectory_;
		std::vector<double> time_vec_;
	};

} // namespace control::integration