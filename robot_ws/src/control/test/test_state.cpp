#include <gtest/gtest.h>
#include "state/state.hpp"

using namespace control::state;

class DummyState : public State
{
public:
    Eigen::VectorXd asVector() const override
    {
        return Eigen::Vector2d(3.0, 4.0);
    }
    
    void fromVector(const Eigen::VectorXd& vec) override
    {
        // dummy implementation
    }
    std::unique_ptr<State> clone() const override
    {
        return std::make_unique<DummyState>();
    }
};

TEST(StateTest, CloneAndAsVectorWorks)
{
    DummyState s;
    Eigen::VectorXd v = s.asVector();
    EXPECT_EQ(v.size(), 2);
    EXPECT_DOUBLE_EQ(v[0], 3.0);
    EXPECT_DOUBLE_EQ(v[1], 4.0);

    auto cloned = s.clone();
    EXPECT_DOUBLE_EQ(cloned->asVector()[0], 3.0);
    EXPECT_DOUBLE_EQ(cloned->asVector()[1], 4.0);
}