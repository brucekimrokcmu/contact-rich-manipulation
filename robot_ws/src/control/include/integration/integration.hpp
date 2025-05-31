#pragma once
#include <cmath>
#include <vector>
#include "control/include/dynamics/ModelBase.hpp"

template <typename State, typename ControlInput>
class IntegrationBase
{
public:
    IntegrationBase(State &x, const control::dynamics::ModelBase &f, const ControlInput &u, double tf, double dt)
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

    virtual ~IntegrationBase() = default;
    virtual void integrate() = 0;
    virtual std::vector<State> integrateTrajectory() = 0;

protected:
    State x_;
    State x0_;
    const control::dynamics::ModelBase &f_;
    double tf_;
    double dt_;
    std::size_t num_steps_;
    std::vector<double> time_vec_;
};
