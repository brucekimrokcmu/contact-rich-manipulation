#include "planning/simple_motion_planner.hpp"

SimpleMotionPlanner::SimpleMotionPlanner()
    : space_(std::make_shared<ompl::base::SE3StateSpace>()), ss(space_)
{
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space_->setBounds(bounds);

    ss.setStateValidityChecker([this](const ompl::base::State *state)
                               { return isStateValid(state); });

    setRandomStartAndGoal();

    ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
}

void SimpleMotionPlanner::setRandomStartAndGoal()
{
    ompl::base::ScopedState<> start(ss.getStateSpace());
    start.random();
    ompl::base::ScopedState<> goal(ss.getStateSpace());
    goal.random();
    
    std::cout << "Start Pose: ";
    start.print(std::cout);

    std::cout << "Goal Pose: ";
    goal.print(std::cout);

    ss.setStartAndGoalStates(start, goal);
}

bool SimpleMotionPlanner::planRRTstar(double timelimit, bool printPath)
{
    if (ss.solve(timelimit))
    {
        std::cout << "Found solution:" << std::endl;
        auto path = ss.getSolutionPath();
        if (printPath)
        {
            path.print(std::cout);
        }

        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("simple_motion_planner"), "Failed to find a solution within the given time limit.");
        return false;
    }
}

bool SimpleMotionPlanner::isStateValid(const ompl::base::State *state) const
{
    const auto *se3State = state->as<ompl::base::SE3StateSpace::StateType>();

    double x = se3State->getX();
    double y = se3State->getY();
    double z = se3State->getZ();

    // TODO: how to set robot joint/torque limitations?

    return true;
}