#include <vector>
#include "my_simple_planner/GetAction.h"
#include "my_simple_planner/GetPath.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "uav_simulator/GetObstacle.h"
#include "uav_simulator/ResetMap.h"
#include "uav_simulator/SetGoal.h"
#include "uav_simulator/State.h"
#include "uav_simulator/Step.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// Test param
double kMaxLeadDistance = 5;
double kMinLeadDistance = 2.0;
double kLeadDistanceFactor = 5.0;
double kMaxMissionNum = 100;
// uav param
double kMaxLinearVelicity = 1.0;
double kMaxAngularVelocity = 1.0;
double kStepTime = 0.1;
// map param
double kTargetDistance = 80;
double kSafeRadius = 0.5;
double kLengthX = 50;
double kLengthY = 100;
double kNumObsMax = 300;
double kNumObsMin = 200;
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
// other
double kGlobalPlanTime = 2.0;
double kSafeDistance = 1.0;

std::vector<double> GetStateVector(const uav_simulator::State state_msg) {
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
  double _linear_velocity = std::sqrt(std::pow(state_msg.twist.linear.x, 2.0) +
                                      std::pow(state_msg.twist.linear.y, 2.0));
  _linear_velocity =
      (_linear_velocity - kMaxLinearVelicity / 2) / (kMaxLinearVelicity / 2);
  _result.push_back(_linear_velocity);
  double _angle_velocity = state_msg.twist.angular.z / kMaxAngularVelocity;
  _result.push_back(_angle_velocity);
  // add target info
  double _target_distance =
      (state_msg.target_distance - 2.5) / 2.5;
  _result.push_back(_target_distance);
  double _target_angle = state_msg.target_angle / M_PI;
  _result.push_back(_target_angle);

  return _result;
}
double Distance(const geometry_msgs::Point &lhs,
                const geometry_msgs::Point &rhs) {
  //
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
              (lhs.y - rhs.y) * (lhs.y - rhs.y));
}
double Distance(const geometry_msgs::Pose &lhs,
                const geometry_msgs::Pose &rhs) {
  //
  return Distance(lhs.position, rhs.position);
}
bool Translate(const geometry_msgs::Point &tr_point,
                          geometry_msgs::Point &point) {
  //
  point.x += tr_point.x;
  point.y += tr_point.y;
  return true;
}
bool Rotate(const double theta, geometry_msgs::Point &point) {
  //
  double _dist = sqrt(point.x * point.x + point.y + point.y);
  double _theta_ori = atan2(point.y, point.x);
  _theta_ori += theta;
  point.x = _dist * cos(_theta_ori);
  point.y = _dist * sin(_theta_ori);
  return true;
}
void Interpolate(nav_msgs::Path &path, const double delta_d) {
  // 
  int32_t _n = path.poses.size();
  nav_msgs::Path _path_t;
  _path_t.header = path.header;
  _path_t.poses.push_back(path.poses.front());
  for (int32_t i = 1; i< _n; i++) {
    double _dist = Distance(path.poses[i-1].pose, path.poses[i].pose);
    double _dist_count = delta_d;
    double _theta = atan2(
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y,
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
    while (_dist_count < _dist) {
      geometry_msgs::Point _point_t;
      _point_t.x = _dist_count;
      _point_t.y = 0;
      _point_t.z = path.poses[i-1].pose.position.z;
      Rotate(_theta, _point_t);
      Translate(path.poses[i-1].pose.position, _point_t);
      geometry_msgs::PoseStamped _pst;
      _pst.pose.orientation = path.poses[i-1].pose.orientation;
      _pst.pose.position = _point_t;
      _path_t.poses.push_back(_pst);
      _dist_count += delta_d;
    }
    _path_t.poses.push_back(path.poses[i]);
  }
  path = _path_t;
}

void GetTrackingPoint(const nav_msgs::Path &path, const uav_simulator::State &cur_state, int32_t &cur_idx) {
  // find tracking point
  geometry_msgs::Point _cur_point;
  _cur_point.x = cur_state.pose.position.x;
  _cur_point.y = cur_state.pose.position.y;
  _cur_point.z = cur_state.pose.position.z;
  double _cur_linear_velocity = sqrt(pow(cur_state.twist.linear.x, 2.0) + pow(cur_state.twist.linear.y, 2.0));
  double _lead_distance = kLeadDistanceFactor * _cur_linear_velocity;
  _lead_distance = std::max(std::min(_lead_distance, kMaxLeadDistance), kMinLeadDistance);
  for (int32_t i = cur_idx, _n = path.poses.size(); i < _n; i++) {
    double _dist_t = Distance(_cur_point, path.poses[i].pose.position);
    if (_dist_t > _lead_distance) break;
    cur_idx = i;
  }
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
  ros::Publisher _global_path_publisher =
      _nh.advertise<nav_msgs::Path>("global_path", 1);
  ros::Publisher _global_path_point_publisher =
      _nh.advertise<visualization_msgs::MarkerArray>("global_path_points", 1);
  ros::Publisher _leading_point_publisher =
      _nh.advertise<visualization_msgs::Marker>("leading_point", 1);
  ros::Publisher _state_pulisher =
      _nh.advertise<uav_simulator::State>("state", 1);

  // _get_action_client.waitForExistence();
  //   _get_path_client.waitForExistence();

  // reset map
  uav_simulator::ResetMap _reset_map_msg;
  _reset_map_msg.request.param.arrive_limit = kArriveLimit;
  _reset_map_msg.request.param.crash_limit = kCrashLimit;
  _reset_map_msg.request.param.intergrate_dt = KIntegrateDt;
  _reset_map_msg.request.param.length_x = kLengthX;
  _reset_map_msg.request.param.length_y = kLengthY;
  _reset_map_msg.request.param.num_obs_max = kNumObsMax;
  _reset_map_msg.request.param.num_obs_min = kNumObsMin;
  _reset_map_msg.request.param.radius_obs_max = kRadiusObsMax;
  _reset_map_msg.request.param.radius_obs_min = kRadiusObsMin;
  _reset_map_msg.request.param.safe_radius = kSafeRadius;
  _reset_map_msg.request.param.target_distance = kTargetDistance;
  _reset_map_msg.request.param.accelerate_rate = 1.0;

  _reset_map_msg.request.sparam.angle_max = kAngleMax;
  _reset_map_msg.request.sparam.angle_min = kAngleMin;
  _reset_map_msg.request.sparam.num_laser = kNumLaser;
  _reset_map_msg.request.sparam.range_max = kRangeMax;
  _reset_map_msg.request.sparam.range_min = kRangeMin;

  bool _flag = _reset_map_client.call(_reset_map_msg);
  if (_flag == false) {
    std::cout << "Reset map failed!" << std::endl;
    // break;
  }

  uav_simulator::State _s0 = _reset_map_msg.response.state;
  std::vector<double> _s0_vector = GetStateVector(_s0);

  // get obstacle information
  uav_simulator::GetObstacle _get_obs_msg;
  _flag = _get_obs_client.call(_get_obs_msg);
  std::vector<geometry_msgs::Point> _obs_pos =
      _get_obs_msg.response.data.obstacles_position;
  std::vector<double> _obs_radius = _get_obs_msg.response.data.obstacles_radius;

  // get global path
  my_simple_planner::GetPath _get_path_msg;
  _get_path_msg.request.start.x = 0;
  _get_path_msg.request.start.y = -kTargetDistance / 2;
  _get_path_msg.request.start.z = 0.5;
  _get_path_msg.request.goal.x = 0;
  _get_path_msg.request.goal.y = kTargetDistance / 2;
  _get_path_msg.request.goal.z = 0.5;
  _get_path_msg.request.map_length_x = kLengthX;
  _get_path_msg.request.map_length_y = kLengthY;
  _get_path_msg.request.obstacle.obstacles_position = _obs_pos;
  _get_path_msg.request.obstacle.obstacles_radius = _obs_radius;
  _get_path_msg.request.plan_time = kGlobalPlanTime;
  _get_path_msg.request.safe_distance = kSafeDistance;

  _flag = _get_path_client.call(_get_path_msg);

  if (_flag == false) {
    std::cout << "Global planning failed!" << std::endl;
  } else {
    std::cout << "Global planning successed." << std::endl;
  }
  // visualize global path
  nav_msgs::Path _global_path;
  _global_path.header.frame_id = "map";
  _global_path.header.stamp = ros::Time::now();
  for (int32_t i = 0; i < _get_path_msg.response.path.size(); i++) {
    geometry_msgs::PoseStamped _ps;
    _ps.pose.position.x = _get_path_msg.response.path[i].x;
    _ps.pose.position.y = _get_path_msg.response.path[i].y;
    _ps.pose.position.z = _get_path_msg.response.path[i].z;
    _ps.pose.orientation.w = 1;
    _global_path.poses.push_back(_ps);
  }
  _global_path_publisher.publish(_global_path);

  // visualize global path points
  visualization_msgs::Marker _mk_msg;
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::SPHERE;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.pose.orientation.w = 1;
  _mk_msg.color.a = 0.5;
  _mk_msg.color.r = 0;
  _mk_msg.color.g = 0.5;
  _mk_msg.color.b = 0;
  _mk_msg.scale.x = 0.4;
  _mk_msg.scale.y = 0.4;
  _mk_msg.scale.z = 0.4;
  for (int32_t i = 0; i < _get_path_msg.response.path.size(); i++) {
    _mk_msg.id = i;
    _mk_msg.pose.position.x = _get_path_msg.response.path[i].x;
    _mk_msg.pose.position.y = _get_path_msg.response.path[i].y;
    _mk_msg.pose.position.z = _get_path_msg.response.path[i].z;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
_global_path_point_publisher.publish(_mk_arr_msg);

  // start tracing
  Interpolate(_global_path, 0.1);
  int32_t _cur_idx = 0;
  bool _done = false;

  while (_done == false) {

    // get leading point
    GetTrackingPoint(_global_path, _s0, _cur_idx);
    // set leading point
    uav_simulator::SetGoal _set_goal_msg;
    _set_goal_msg.request.position = _global_path.poses[_cur_idx].pose.position;
    _set_goal_client.call(_set_goal_msg);
    _s0 = _set_goal_msg.response.state;
    _s0_vector = GetStateVector(_s0);

    _state_pulisher.publish(_s0);
    // visualize leading point
    visualization_msgs::Marker _mk_msg;
    _mk_msg.header.frame_id = "map";
    _mk_msg.header.stamp = ros::Time::now();
    _mk_msg.action = _mk_msg.ADD;
    _mk_msg.color.a = 0.5;
    _mk_msg.color.r = 1.0;
    _mk_msg.color.g = 0;
    _mk_msg.color.b = 0;
    _mk_msg.id = 0;
    _mk_msg.scale.x = 0.25;
    _mk_msg.scale.y = 0.25;
    _mk_msg.scale.z = 0.25;
    _mk_msg.pose.position = _global_path.poses[_cur_idx].pose.position;
    _mk_msg.pose.orientation.w = 1.0;
    _mk_msg.type = _mk_msg.SPHERE;
    _leading_point_publisher.publish(_mk_msg);
    // get action
    my_simple_planner::GetAction _get_action_msg;
    _get_action_msg.request.state = _s0_vector;
    _get_action_client.call(_get_action_msg);
    _get_action_msg.response.action;

    // step
    uav_simulator::Step _step_msg;
    _step_msg.request.control.linear_velocity =
        (_get_action_msg.response.action[0] + 1) / 2 * kMaxLinearVelicity;
    _step_msg.request.control.yaw_rate =
        _get_action_msg.response.action[1] * kMaxAngularVelocity;
    _step_msg.request.step_time = kStepTime;
    _step_client.call(_step_msg);
    
    // 
    bool _is_crash = _step_msg.response.is_crash;
    bool _is_arrive = _step_msg.response.is_arrive;
    _done  = (_is_arrive || _is_crash);

    if (_is_crash) {
      std::cout << "Mission: Crashed!" << std::endl;
    }
    if (_is_arrive) {
      std::cout << "Mission: Arrived!" << std::endl;
    }

    // update state
    _s0 = _step_msg.response.state;
    _s0_vector = GetStateVector(_s0);

    // delete leading point
    _mk_msg.header.stamp = ros::Time::now();
    _mk_msg.action = _mk_msg.DELETE;
    _leading_point_publisher.publish(_mk_msg);
  }

  ros::spin();

  return 0;
}