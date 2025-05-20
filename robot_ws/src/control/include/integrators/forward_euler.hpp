#pragma once

#include "integrator_base.hpp"

template <typename State, typename Dynamics>
class ForwardEuler : public IntegratorBase<State, Dynamics>
{
public:
    using Base = IntegratorBase<State, Dynamics>;
    ForwardEuler(State &x, const Dynamics &f, double tf, double dt)
        : Base(x, f, tf, dt)
    {
    }

    void integrate() override
    {
        for (std::size_t i = 1; i < this->num_steps_; ++i)
        {
            double t = this->time_vec_[i - 1];
            this->x_ = eulerStep(this->x_, t);
        }
    }

    std::vector<State> integrateTrajectory() override
    {
        std::vector<State> trajectory(this->num_steps_);
        trajectory[0] = this->x0_;
        for (std::size_t i = 1; i < this->num_steps_; ++i)
        {
            double t = this->time_vec_[i - 1];
            trajectory[i] = eulerStep(trajectory[i - 1], t);
        }

        return trajectory;
    }

private:
    State eulerStep(const State &x, double t) const
    {
        const double dt = this->dt_;
        const auto &f = this->f_;

        return x + dt * f(x, t);
    }
};
