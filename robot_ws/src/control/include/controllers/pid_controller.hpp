#pragma once
#include <limits>

namespace control::controllers
{

    class PID
    {
    public:
        PID(double kp, double ki, double kd);
        void setKp(double kp);
        void setKi(double ki);
        void setKd(double kd);
        void setGain(double kp, double ki, double kd);
        void reset();
        double compute(double error, double dt);

    private:
        double kp_, ki_, kd_;
        double integral_;
        double integral_limit_;
        double prev_error;
    };

} // namespace control::controllers