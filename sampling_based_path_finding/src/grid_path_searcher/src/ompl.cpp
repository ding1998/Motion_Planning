#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <ros/ros.h>

#include "graph_searcher.h"

/*
    确定我们计划的空间：SE(3)
    从可用的状态空间中选择一个相应的状态空间，或者实现一个。对于 SE(3)，ompl::base::SE3StateSpace是合适的。
    由于 SE(3) 包含一个 R 3分量，我们需要定义边界。
    定义状态有效性的概念。
    定义开始状态和目标表示。
    ...
*/

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    // const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    // const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    // return (const void *)rot != (const void *)pos;
    return 1;
}

void plan()
{
    //构建状态空间
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    
    //设置bounds for SE3
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    //从状态空间space中构建一个状态信息的实例
    auto si(std::make_shared<ob::SpaceInformation>(space));
    //设置这个空间内的状态有效性检查
    si->setStateValidityChecker(isStateValid);
    //创建随机开始状态
    ob::ScopedState<> start(space);
    start[0] = -0.5;
    start[1] = 0.0;
    start[2] = 0.0;
    //start.random();
    //创建随机结束状态
    ob::ScopedState<> goal(space);
    goal[0] = 0.5;
    goal[1] = 0.0;
    goal[2] = 0.0;
    //goal.random();

    //创建一个问题实例
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    //设置开始和结束状态
    pdef->setStartAndGoalStates(start, goal);
    //为该空间创建一个规划器
    auto planner(std::make_shared<og::RRTConnect>(si));
    //ob::PlannerPtr planner(new og::RRTstar(si));
    //设置规划器的问题实例
    planner->setProblemDefinition(pdef);
    //开始
    printf("111111-------------------------------------");
    planner->setup();
    //打印状态空间设置
    si->printSettings(std::cout);
    printf("222222-------------------------------------");
    //打印问题设置
    pdef->print(std::cout);
    printf("333333-------------------------------------");
    //规划器求解 时间限制1.0s
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        //从pedf中获取目标的表示
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        //打印路径导screen
        path->print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}
//another way
void planWithSimpleSetup()
{
    //construct the SE3 state space
    auto space(std::make_shared<ob::SE3StateSpace>());
    //set bounds for SE3
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    //define a simple setup class
    og::SimpleSetup ss(space);
    //set state validity for the space
    ss.setStateValidityChecker([](const ob::State *state)
                               { return isStateValid(state); });
    //create a random start state
    ob::ScopedState<> start(space);
    start.random();
    //create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();
    //set start and goal state
    ss.setStartAndGoalStates(start, goal);
    //this is optional,if it is used ,we can get more information
    ss.setup();
    ss.print();
    //attempt to solve the problem within 1s
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found soltion:" << std::endl;
        //print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl");
    ros::NodeHandle nh("~");

    std::cout << "OMPLversion:" << OMPL_VERSION << std::endl;

    plan();
    std::cout << std::endl
              << std::endl;
    //another way with simplesetup
    // planWithSimpleSetup();

    return 0;
}
