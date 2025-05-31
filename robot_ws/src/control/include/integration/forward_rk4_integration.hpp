#pragma once

#include "integration_base.hpp"

template <typename State, typename Dynamics>
class ForwardRK4 : public IntegratorBase<State, Dynamics>
{
public:
    using Base = IntegratorBase<State, Dynamics>;
    ForwardRK4(State &x, const Dynamics &f, double tf, double dt)
        : Base(x, f, tf, dt)
    {
    }

    void integrate() override
    {
        for (std::size_t i = 1; i < this->num_steps_; ++i)
        {
            double t = this->time_vec_[i - 1];
            this->x_ = rk4Step(this->x_, t)
        }
    }

    std::vector<State> integrateTrajectory() override
    {
        std::vector<State> trajectory(this->num_steps_);
        trajectory[0] = this->x0_;

        for (std::size_t i = 1; i < this->num_steps_; ++i)
        {
            double t = this->time_vec_[i - 1];
            trajectory[i] = rk4Step(trajectory[i - 1], t);
        }

        return trajectory;
    }

private:
    State rk4Step(const State &x, double t) const
    {
        const double dt = this->dt_;
        const auto &f = this->f_;

        State k1 = f(x, t);
        State k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt);
        State k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt);
        State k4 = f(x + dt * k3, t + dt);

        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
};