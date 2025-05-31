#pragma once

namespace control::controllers 
{

class PID 
{
public:
    PID(double kp, double ki, double kd)
    :  kp_(kp),
       ki_(ki),
       kd_(kd)
    {
    }


private:
    double kp_, ki_, kd_;
    double integral;

};

} // namespace control::controllers