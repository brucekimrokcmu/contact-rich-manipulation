#include <gtest/gtest.h>
#include "control_input/control_input.hpp"

using namespace control::control_input;

class DummyControlInput : public ControlInput
{
public:
    Eigen::VectorXd asVector() const override
    {
        return Eigen::Vector2d(1.0, 2.0);
    }

    void fromVector(const Eigen::VectorXd &vec) override
    {
        // dummy implementation
    }
};


TEST(ControlInputTest, CanBeInstantiatedAndUsed)
{
    DummyControlInput input;
    Eigen::VectorXd vec = input.asVector();
    EXPECT_EQ(vec.size(), 2);
    EXPECT_DOUBLE_EQ(vec[0], 1.0);
    EXPECT_DOUBLE_EQ(vec[1], 2.0);
}