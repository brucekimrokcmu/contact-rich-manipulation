#pragma once

namespace control::utils
{
    constexpr double PI = 3.14159265358979323846;

    inline double deg2rad(double deg)
    {
        return deg * PI / 180.0;
    }

    inline double rad2deg(double rad)
    {
        return rad * 180.0 / PI;
    }

} // namespace control::utils
