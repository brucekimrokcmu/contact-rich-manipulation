#pragma once
#include "control_input/control_input.hpp"

namespace control::control_input
{
    class AckermannControlInput : public ControlInput
    {
    public:
        AckermannControlInput(double a, double phi)
            : a_(a),
              phi_(phi)
        {
        }

        Eigen::Vector2d asVector() const
        {
            return Eigen::Vector2d(a_, phi_).finished();
        }

        void fromVector(const Eigen::Vector2d &vec)
        {
            a_ = vec(0);
            phi_ = vec(1);
        }

        double a() const { return a_; }
        double phi() const { return phi_; }

    private:
        double a_, phi_;
    };

} // namespace control::control_input
