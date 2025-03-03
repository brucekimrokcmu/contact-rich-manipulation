#pragma once
#include <iostream>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

class SimpleMotionPlanner
{
public:
    SimpleMotionPlanner();
    void setRandomStartAndGoal();
    bool planRRTstar(double timelimit, bool printPath);

private:
    bool isStateValid(const ompl::base::State *state) const;
    std::shared_ptr<ompl::base::SE3StateSpace> space_;
    ompl::geometric::SimpleSetup ss;
};
