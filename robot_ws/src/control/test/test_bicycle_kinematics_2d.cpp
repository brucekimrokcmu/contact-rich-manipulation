#include <gtest/gtest.h>
#include "control/include/model/bicycle_kinemtics.hpp"
#include "control/include/model/control_input.hpp"
#include "control/include/state/bicycle_state_2d.hpp"
#include "control/utils/math_utils.hpp"

using namespace control::state;
using namespace control::model;

TEST(BicycleKinematicsTest, BicycleKinematics)
{
	BicycleKinematics model;

	// Define initial state
	x_0 = 0.0;
	y_0 = 0.0;
	theta_0 = 30.0;
	delta_0 = 0.0;
	v_0 = 0.0;

	BicycleState2D s(x_0, y_0, theta_0, delta_0, v_0);

	// Define input
	ControlInput input;
	input.velocity = 1.0;  // 1 m/s
	input.steering_angle = 0.1;  // 0.1 radians

	// Define time step
	double dt = 0.1;  // 0.1 seconds

	// Perform state update
	BicycleState updated_state = model.update(initial_state, input, dt);

	// Check updated state
	EXPECT_NEAR(updated_state.x, 0.1, 1e-5);
	EXPECT_NEAR(updated_state.y, 0.0, 1e-5);
	EXPECT_NEAR(updated_state.theta, 0.01, 1e-5);

}