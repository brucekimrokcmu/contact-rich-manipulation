#include <gtest/gtest.h>
#include "dynamics/dynamics.hpp"
#include "state/state.hpp"
#include "control_input/control_input.hpp"

using namespace control;

class DummyState : public state::State
{
public:
    Eigen::VectorXd asVector() const override { return Eigen::Vector2d(0, 0); }
    void fromVector(const Eigen::VectorXd &) override {}
    std::unique_ptr<state::State> clone() const override { return std::make_unique<DummyState>(); }
};

class DummyInput : public control_input::ControlInput
{
public:
    Eigen::VectorXd asVector() const override { return Eigen::Vector2d(0, 0); }
    void fromVector(const Eigen::VectorXd &) override {}
};

class DummyDynamics : public dynamics::Dynamics
{
public:
    std::unique_ptr<state::State> f(const state::State &, const control_input::ControlInput &) const override
    {
        return std::make_unique<DummyState>();
    }
};

TEST(DynamicsTest, CanComputeNextState)
{
    DummyState s;
    DummyInput u;
    DummyDynamics dyn;
    auto next = dyn.f(s, u);
    EXPECT_NE(next, nullptr);
}
