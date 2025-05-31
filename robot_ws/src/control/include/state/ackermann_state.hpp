#pragma once
#include "state.hpp"

namespace control::state
{
    class AckermannState : public State
    {
    public:
        AckermannState(double x, double y, double theta, double delta, double v)
            : x_(x),
              y_(y),
              theta_(theta),
              delta_(delta),
              v_(v)
        {
        }

        Eigen::VectorXd asVector() const override
        {
            return Eigen::VectorXd { (Eigen::VectorXd(6) << x, y, theta, delta, v).finished() }
        }

        void fromVector(const Eigen::VectorXd &vec) override
        {
            x_ = vec(0);
            y_ = vec(1);
            theta_ = vec(2);
            delta_ = vec(3);
            v_ = vec(4);
        }

        std::unique_ptr<State> clone() const override
        {
            return std::make_unique<AckermannState>(*this);
        }

        double x() const { return x_; }
        double y() const { return y_; }
        double theta() const { return theta_; }
        double delta() const { return delta_; }
        double v() const { return v_; }

    private:
        double x_, y_, theta_, delta_, v_;
    }

} // namespace control::state
