#pragma once

#include "integration_base.hpp"

template <typename State, typename ControlInput>
class ImplicitMidpoint : public IntegrationBase<State, ControlInput>
{
public:
    using Base = IntegratorBase<State, ControlInput>;
    ImplicitMidpoint(State &x, const Dynamics &f, double tf, double dt)
        : Base(x, f, u, tf, dt)
    {
    }

    void integrate() override
    {
        for (std::size_t i = 1; i < this->num_steps_; ++i)
        {
            double t = this->time_vec_[i - 1];
            this->x_ = step(this->x_, t);
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
        const ControlInput &u = this->u_;

        // Implicit midpoint method: x_{n+1} = x_n + dt * f((x_n + x_{n+1}) / 2, u)
        State x_next = x; // Initial guess
        for (int iter = 0; iter < 10; ++iter) // Simple fixed-point iteration
        {
            State mid_state = (x + x_next) / 2;
            x_next = x + dt * f(mid_state, u);
        }

        return x_next;
    }

      
};
