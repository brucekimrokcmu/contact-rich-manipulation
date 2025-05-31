#pragma once
#include <Eigen/Dense>
#include <memory>

namespace control::state
{
    class State
    {
    public:
        virtual ~State() = default;
        virtual Eigen::VectorXd asVector() const = 0;
        virtual void fromVector(const Eigen::VectorXd &vec) = 0;
        virtual std::unique_ptr<State> clone() const = 0;
    };
} // namespace control::state
