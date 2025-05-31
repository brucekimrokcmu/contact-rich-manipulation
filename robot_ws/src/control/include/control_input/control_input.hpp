#pragma once
#include <Eigen/Dense>

namespace control::control_input
{
    class ControlInput
    {
    public:
        virtual ~ControlInput() = default;
        virtual Eigen::VectorXd asVector() const = 0;
        virtual void fromVector(const Eigen::VectorXd &vec) = 0;
    };
} // namespace control::control_input
