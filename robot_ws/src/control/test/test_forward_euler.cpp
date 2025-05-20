#include <gtest/gtest.h>
#include "integrators/forward_euler.hpp"

double simple_dynamics(const double &x, double t)
{
    return 2.0; // dx/dt = 2
}

TEST(ForwardEulerTest, SimpleIntegration)
{
    double x0 = 1.0;
    double tf = 1.0;
    double dt = 0.5;

    ForwardEuler<double, decltype(&simple_dynamics)> integrator(x0, simple_dynamics, tf, dt);

    integrator.integrate();

    EXPECT_NEAR(integrator.getState(), 3.0, 1e-6);

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}