#pragma once
#include <cmath>
#include <vector>

template <typename State, typename Dynamics>
class IntegratorBase
{
public:
    IntegratorBase(State &x, const Dynamics &f, double tf, double dt)
        : x_(x),
          x0_(x),
          f_(f),
          tf_(tf),
          dt_(dt)
    {
        num_steps_ = static_cast<std::size_t>(std::round(tf_ / dt_)) + 1;
        time_vec_.reserve(num_steps_);
        for (std::size_t i = 0; i < num_steps_; ++i)
        {
            time_vec_.push_back(i * dt_);
        }
    }

    virtual ~IntegratorBase() = default;
    virtual void integrate() = 0;
    virtual std::vector<State> integrateTrajectory() = 0;
    // virtual const std::vector<double> &time_vec() const { return time_vec; };

    State getState() const { return this->x_; }

protected:
    State x_;
    State x0_;
    const Dynamics &f_;
    double tf_;
    double dt_;
    std::size_t num_steps_;
    std::vector<double> time_vec_;
};
