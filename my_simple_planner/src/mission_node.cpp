#include "ros/ros.h"
#include "uav_simulator/ResetMap.h"
#include "uav_simulator/Step.h"
#include "uav_simulator/SetGoal.h"
#include "uav_simulator/GetObstacle.h"
#include "my_simple_planner/GetAction.h"
#include "my_simple_planner/GetPath.h"
#include "uav_simulator/State.h"
#include <vector>

// Test param
double kLeadDistance = 5;
double kMaxMissionNum = 100;
double kMaxStepNum = 100;
// uav param
double kMaxLinearVelicity = 1.0;
double kMaxAngularVelocity = 1.0;
double kStepTime = 0.2;
// map param
double kTargetDistance = 80;
double kSafeRadius = 0.5;
double kLengthX = 150;
double kLengthY = 150;
double kNumObsMax = 100;
double kNumObsMin = 50;
double kRadiusObsMax = 2.0;
double kRadiusObsMin = 0.25;
double kCrashLimit = 0.25;
double kArriveLimit = 0.25;
double KIntegrateDt = 0.02;
// sensor param
double kRangeMax = 5.0;
double kRangeMin = 0.15;
double kAngleMax = 2 * M_PI / 3;
double kAngleMin = -2 * M_PI / 3;
double kNumLaser = 40;

std::vector<double> GetStateVector(const uav_simulator::State state_msg){
    //
    std::vector<double> _result;
    // add obstacle info
    for (int32_t i = 0; i < state_msg.scan.ranges.size(); i++) {
        double _interval_length =
          state_msg.scan.range_max - state_msg.scan.range_min;
        _result.push_back((state_msg.scan.ranges[i] - _interval_length / 2) /
                          (_interval_length / 2));
    }
    // add velocity info
    double _linear_velocity =
        std::sqrt(std::pow(state_msg.twist.linear.x, 2.0) +
                  std::pow(state_msg.twist.linear.y, 2.0));
    _linear_velocity =
        (_linear_velocity - kMaxLinearVelicity / 2) / (kMaxLinearVelicity / 2);
    _result.push_back(_linear_velocity);
    double _angle_velocity = state_msg.twist.angular.z / kMaxAngularVelocity;
    _result.push_back(_angle_velocity);
    // add target info
    double _target_distance =
        (state_msg.target_distance - kTargetDistance / 2) /
        (kTargetDistance / 2);
    _result.push_back(_target_distance);
    double _target_angle = (state_msg.target_angle - M_PI_2) / M_PI_2;
    _result.push_back(_target_angle);
    
    return _result;
}

int32_t main(int32_t argc, char *argv[]) {

  srand(time(NULL));
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "mission_node");
  ros::NodeHandle _nh;

  // client
  ros::ServiceClient _reset_map_client =
      _nh.serviceClient<uav_simulator::ResetMap>("reset_map");
  ros::ServiceClient _step_client =
      _nh.serviceClient<uav_simulator::Step>("step");
  ros::ServiceClient _set_goal_client =
      _nh.serviceClient<uav_simulator::SetGoal>("set_goal");
  ros::ServiceClient _get_obs_client =
      _nh.serviceClient<uav_simulator::GetObstacle>("get_obstacle");
  ros::ServiceClient _get_action_client =
      _nh.serviceClient<my_simple_planner::GetAction>("get_action");
  ros::ServiceClient _get_path_client =
      _nh.serviceClient<my_simple_planner::GetPath>("get_path");

//   _get_action_client.waitForExistence();
//   _get_path_client.waitForExistence();

  //
  uav_simulator::ResetMap _reset_map_msg;
  _reset_map_msg.request.param.arrive_limit = kArriveLimit;
  _reset_map_msg.request.param.crash_limit = kCrashLimit;
  _reset_map_msg.request.param.intergrate_dt = KIntegrateDt;
  _reset_map_msg.request.param.length_x  = kLengthX;
  _reset_map_msg.request.param.length_y  = kLengthY;
  _reset_map_msg.request.param.num_obs_max = kNumObsMax;
  _reset_map_msg.request.param.num_obs_min = kNumObsMin;
  _reset_map_msg.request.param.radius_obs_max = kRadiusObsMax;
  _reset_map_msg.request.param.radius_obs_min = kRadiusObsMin;
  _reset_map_msg.request.param.safe_radius = kSafeRadius;
  _reset_map_msg.request.param.target_distance = kTargetDistance;
  //
  uav_simulator::GetObstacle _get_obs_msg;
  //
  uav_simulator::Step _step_msg;
  _step_msg.request.step_time = kStepTime;
  //
  my_simple_planner::GetPath _get_path_msg;
  _get_path_msg.request.start.x = 0;
  _get_path_msg.request.start.y = -kTargetDistance/2;
  _get_path_msg.request.start.x = 0.5;
  _get_path_msg.request.goal.x = 0;
  _get_path_msg.request.goal.y = kTargetDistance/2;
  _get_path_msg.request.goal.x = 0.5;

    // reset map
    bool _flag = _reset_map_client.call(_reset_map_msg);
    // if (_flag == false) {
    //   std::cout << "Reset map failed!" << std::endl;
    //   // break;
    // } 
    // uav_simulator::State _s0 = _reset_map_msg.response.state;
    // std::vector<double> _s0_vector = GetStateVector(_s0);
    // // get obstacle info
    // _flag = _get_obs_client.call(_get_obs_msg);
    // std::vector<geometry_msgs::Point> _obs_pos =
    //     _get_obs_msg.response.data.obstacles_position;
    // std::vector<double> _obs_radius =
    //     _get_obs_msg.response.data.obstacles_radius;
    

    // // get global path
    // _get_path_msg.request.obstacle.obstacles_position = _obs_pos;
    // _get_path_msg.request.obstacle.obstacles_radius = _obs_radius;
    // _flag = _get_path_client.call(_get_path_msg);
    // std::vector<geometry_msgs::Point> _path = _get_path_msg.response.path;


  // for (int32_t i=0; i < kMaxMissionNum; i++) {
  //   // reset map
  //   bool _flag = _reset_map_client.call(_reset_map_msg);
  //   if (_flag == false) {
  //     std::cout << "Reset map failed!" << std::endl;
  //     break;
  //   } 
  //   uav_simulator::State _s0 = _reset_map_msg.response.state;
  //   // get obstacle info
  //   _flag = _get_obs_client.call(_get_obs_msg);
  //   std::vector<geometry_msgs::Point> _obs_pos =
  //       _get_obs_msg.response.data.obstalces_position;
  //   std::vector<double> _obs_radius =
  //       _get_obs_msg.response.data.obstacles_radius;
  //   // get global path
  //   _get_path_msg.request.obstacle.obstalces_position = _obs_pos;
  //   _get_path_msg.request.obstacle.obstacles_radius = _obs_radius;
  //   _flag = _get_path_client.call(_get_path_msg);
  //   std::vector<geometry_msgs::Point> _path = _get_path_msg.response.path;
  //   // start to trace

  //   //

  // }

  ros::spin();

  return 0;
}