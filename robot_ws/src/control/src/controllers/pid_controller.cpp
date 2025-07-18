#include "pid_controller.hpp"

namespace control::controllers
{

    PID::PID(double kp, double ki, double kd)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          integral_(0.0),
          prev_error_(0.0),
          integral_limit_(std::numeric_limits<double>::max())
    {
    }

    void PID::setKp(double kp) { kp_ = kp; }
    void PID::setKi(double ki) { ki_ = ki; }
    void PID::setKd(double kd) { kd_ = kd; }

    void PID::setGain(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void PID::reset()
    {
        integral_ = 0.0;
        prev_error = 0.0;
    }

    double PID::compute(double error, double dt)
    {
        integral_ += error * dt;
        if (integral_ > integral_limit_)
        {
            integral_ = integral_limit_;
        }
        else if (integral_ < -integral_limit_)
        {
            integral_ = -integral_limit_;
        }

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

} // namespace control::controllers