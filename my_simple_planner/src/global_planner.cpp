#include "ros/ros.h"
#include "uav_simulator/ResetMap.h"
#include "my_simple_planner/GetPath.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <vector>
#include "geometry_msgs/Point.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<geometry_msgs::Point> obs_pos_;
std::vector<double> obs_radius_;

double Distance(const geometry_msgs::Point &lhs,
                           const geometry_msgs::Point &rhs) {
  //
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
              (lhs.y - rhs.y) * (lhs.y - rhs.y));
}

bool IsStateValid(const ob::State *state) {
  //
  // get current point
  geometry_msgs::Point _cur_point;
  const ob::RealVectorStateSpace::StateType *_state2D =
      state->as<ob::RealVectorStateSpace::StateType>();
  _cur_point.x = (*_state2D)[0];
  _cur_point.y = (*_state2D)[1];
  for (int32_t i = 0; i < obs_pos_.size(); i++) {
    if (Distance(obs_pos_[i], _cur_point) < obs_radius_[i]) {
      return true;
    }
  }
  return false;
}

bool GetPath(my_simple_planner::GetPath::Request &req,
             my_simple_planner::GetPath::Response &resp) {
  //
  // set obstacle info in global
  obs_pos_ = req.obstacle.obstacles_position;
  obs_radius_  = req.obstacle.obstacles_radius;
  // construct the state space we are planning in
  auto _space(std::make_shared<ob::RealVectorStateSpace>(2));
  // set the bounds for the R^2 part of SE(2)
  ob::RealVectorBounds _bounds(2);
  _bounds.setHigh(0, req.map_length_x/2);
  _bounds.setHigh(1, req.map_length_y/2);
  _bounds.setLow(0, -req.map_length_x/2);
  _bounds.setLow(1, -req.map_length_y/2);
  _space->setBounds(_bounds);
  // construct an instance of  space information from this state space
  auto _si(std::make_shared<ob::SpaceInformation>(_space));
  // set state validity checking for this space
  _si->setStateValidityChecker(IsStateValid);
  // create a start state
  ob::ScopedState<> _start(_space);
  _start[0] = req.start.x;
  _start[1] = req.start.y;
  // create a start state
  ob::ScopedState<> _goal(_space);
  _goal[0] = req.start.x;
  _goal[1] = req.start.y;
  // create a problem instance
  auto _pdef(std::make_shared<ob::ProblemDefinition>(_si));
  // set the start and goal states
  _pdef->setStartAndGoalStates(_start, _goal);
  // create a planner for the defined space
  auto _planner(std::make_shared<og::RRTstar>(_si));
  // set the problem we are trying to solve for the planner
  _planner->setProblemDefinition(_pdef);
  // perform setup steps for the planner
  _planner->setup();
  // print the settings for this space
  _si->printSettings(std::cout);
  // print the problem settings
  _pdef->print(std::cout);
  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus _solved = _planner->ob::Planner::solve(req.plan_time);

  if (_solved) {
    og::PathGeometric* _path = _pdef->getSolutionPath()->as<og::PathGeometric>();
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    _path->print(std::cout);
    // construct response msg
    resp.success  = true;
    std::vector<geometry_msgs::Point> _points;
    for (int32_t i = 0; i < _path->getStateCount(); i++) {
      const ob::RealVectorStateSpace::StateType *state =
          _path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
      geometry_msgs::Point _point;
      _point.x  = (*state)[0];
      _point.y  = (*state)[1];
      _points.push_back(_point);
    }
    resp.path = _points;
  }
  else {
    std::cout << "No solution found" << std::endl;
    resp.success = false;
  }

  return true;
}

int32_t main(int32_t argc, char *argv[]) {
  srand(time(NULL));
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle _nh;

  ros::ServiceServer _get_path_server =
      _nh.advertiseService("get_path", GetPath);

  ros::spin();

  return 0;
}