#pragma once

#include "integration_base.hpp"

namespace control::integration
{
    using control::control_input::ControlInput;
    using control::dynamics::Dynamics;
    using control::state::State;

    class ExplicitMidpoint : public IntegrationBase
    {
    public:
        ExplicitMidpoint(std::unique_ptr<State> x0, const Dynamics &f, double tf, double dt)
            : integrationBase(std::move(x0), f, tf, dt)
        {
        }

        void integrate(const ControlInput &u) override
        {
            for (std::size_t i = 1; i < this->num_steps_; ++i)
            {
                step(*x_, u);
            }
        }

        std::vector<State> integrateTrajectory() override
        {
            std::vector<State> trajectory(this->num_steps_);
            trajectory[0] = this->x0_;
            for (std::size_t i = 1; i < this->num_steps_; ++i)
            {
                double t = this->time_vec_[i - 1];
                trajectory[i] = step(trajectory[i - 1], t);
            }

            return trajectory;
        }

    private:
        State step(const State &x, double t) const
        {
            const double dt = this->dt_;
            const auto &f = this->f_;

            x_m = x + 0.5 * dt * f(x, t)

                                     return x +
                  dt * f(x_m, t);
        }
    };

} // namespace control::integration