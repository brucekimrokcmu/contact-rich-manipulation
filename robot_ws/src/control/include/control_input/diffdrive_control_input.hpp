#pragma once
#include "control_input/control_input.hpp"

namespace control::control_input
{
    class DiffDriveControlInput : ControlInput
    {
    public:
        DiffDriveControlInput(double v, double omega)
            : v_(v),
              omega_(omega)
        {
        }

        ControlInputType type() const override
        {
            return ControlInputType::DiffDrive;
        }
        std::unique_ptr<ControlInput> clone() const override
        {
            return std::make_unique<DiffDriveControlInput>(*this);
        }

        std::size_t size() const override
        {
            return 2; // v and omega
        }

        Eigen::VectorXd asVector() const override
        {
            return (Eigen::Vector2d() << v_, omega_).finished();
        }

        void fromVector(const Eigen::VectorXd &vec) override
        {
            v_ = vec(0);
            omega_ = vec(1);
        }

        double v() const { return v_; }
        double omega() const { return omega_; }

    private:
        double v_;
        double omega_;
    };

} // namespace control::control_input