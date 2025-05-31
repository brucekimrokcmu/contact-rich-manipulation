#pragma once
#include "model_base.hpp"
#include "control/state/bicycle_state_2d.hpp"
#include <cmath>
#include <memory>

namespace control::model
{
    class BicycleKinematics2D : public ModelBase
    {
    public:
        // TODO: Assume center as a desired point
        template <typename StateType> // TODO: check if template is desirable
        std::unique_ptr<control::state::State> f(const StateType &state, const control::model::ControlInput &input) const override
        {
            
            double x = state.x();
            double y = state.y();
            double theta = state.theta();
            double delta = state.delta();
            double v = state.v()
            double L = wheelbase_;

            double x_dot = v * std::cos(delta + theta);
            double y_dot = v * std::sin(delta + theta);
            double theta_dot = v * std::sin(delta) / L;
            double delta_dot = input.phi();
            double v_dot = input.a();

            return std::make_shared<control::state::BicycleState2D>(x_dot, y_dot, theta_dot, delta_dot)
        }

    private:
        double wheelbase_;
    };

} // namespace control::model
