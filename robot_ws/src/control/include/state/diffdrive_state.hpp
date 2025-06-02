#pragma once
#include "state/state.hpp:

namespace control::state
{
    class DiffDriveState : public State
    {
    public:
        DiffDriveState(double x, double y, double theta)
            : x_(x),
              y_(y),
              theta_(theta)
        {
        }

        Eigen::VectorXd asVector() const override
        {
            return Eigen::VectorXd { (Eigen::VectorXd(3) << x_, y_, theta_).finished() }
        }

        void fromVector(const Eigen::VectorXd &vec) override
        {
            x_ = vec(0);
            y_ = vec(1);
            theta_ = vec(2);
        }

        std::unique_ptr<State> clone() const override
        {
            return std::make_unique<DiffDriveState>(*this);
        }

        double x() const { return x_; }
        double y() const { return y_; }
        double theta() const { return theta_; }

    private:
        double x_, y_, theta_;
    }

} // namespace control::state
