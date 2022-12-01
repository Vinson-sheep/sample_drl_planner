#include "Simulator.h"
#include "angles/angles.h"

Simulator::Simulator() {
  //
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh("~");

  // uav state
  _private_nh.param("max_linear_velocity", max_linear_velocity_, 1.0);
  _private_nh.param("max_angular_velocity", max_angular_velocity_, 1.0);
  _private_nh.param("crash_limit", crash_limit_, 0.2);
  _private_nh.param("arrive_limit", arrive_limit_, 0.2);
  _private_nh.param("angle_max", angle_max_, 2 * M_PI / 3);
  _private_nh.param("angle_min", angle_min_, -2 * M_PI / 3);
  _private_nh.param("range_max", range_max_, 3.0);
  _private_nh.param("range_min", range_min_, 0.15);
  _private_nh.param("num_laser", num_laser_, 40);
  _private_nh.param("flight_height", flight_height_, 0.5);
  _private_nh.param("intergrate_dt", intergrate_dt_, 0.02);
  _private_nh.param("accelerate_rate", accelerate_rate_, 1.0);
  _private_nh.param("state_update_factor", state_update_factor_, 0.1);
  state_.pose.orientation.w = 1.0;

  // grid map
  _private_nh.param("target_distance", target_distance_, 8.0);
  _private_nh.param("safe_radius", safe_radius_, 0.5);
  _private_nh.param("length_x", length_x_, 15.0);
  _private_nh.param("length_y", length_y_, 15.0);
  _private_nh.param("num_obs_max", num_obs_max_, 20);
  _private_nh.param("num_obs_min", num_obs_min_, 2);
  _private_nh.param("radius_obs_max", radius_obs_max_, 2.0);
  _private_nh.param("radius_obs_min", radius_obs_min_, 0.25);
  _private_nh.param("num_obs_max_extra", num_obs_max_extra_, 5);
  _private_nh.param("num_obs_min_extra", num_obs_min_extra_ , 2);
  _private_nh.param("radius_obs_max_extra", radius_obs_max_extra_, 0.3);
  _private_nh.param("radius_obs_min_extra", radius_obs_min_extra_, 0.25);
  _private_nh.param("vibration_distance_extra", vibration_distance_extra_, 0.5);
  start_goal_.resize(2);
  start_goal_[0].x = 0;
  start_goal_[0].y = -target_distance_ / 2;
  start_goal_[0].z = flight_height_;
  start_goal_[1].x = 0;
  start_goal_[1].y = target_distance_ / 2;
  start_goal_[1].z = flight_height_;

  // path
  global_path_.header.frame_id = "map";

  // tracking 
  _private_nh.param("lead_distance_factor", lead_distance_factor_, 3.0);
  _private_nh.param("max_leading_distance", max_leading_distance_, 3.0);
  _private_nh.param("min_leading_distance", min_leading_distance_, 1.0);
  _private_nh.param("num_tracking_point", num_tracking_point_, 5);  

  // reward
  _private_nh.param("distance_reward_allocation_factor",
                    distance_reward_allocation_factor_, 1.0);
  _private_nh.param("tracking_reward_include",
                    tracking_reward_include_, true);

  // Subscriber
  rviz_goal_sub_ = _nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, &Simulator::RvizGoalCB, this);

  // Publisher
  visual_obs_publisher_ =
      _nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
  visual_obs_extra_publisher_ =
      _nh.advertise<visualization_msgs::MarkerArray>("obstacles_extra", 10);
  visual_local_goals_publisher_ =
      _nh.advertise<visualization_msgs::MarkerArray>("local_goals", 10);
  visual_start_goal_publisher_ = 
      _nh.advertise<visualization_msgs::MarkerArray>("start_goal", 10);
  laser_scan_publisher_ =
      _nh.advertise<sensor_msgs::LaserScan>("laser_scan", 10);
  visual_path_publisher_ = 
      _nh.advertise<nav_msgs::Path>("global_path", 1);
  visual_waypoints_publisher_ = 
      _nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);

  // Servicer
  add_obs_server_ =
      _nh.advertiseService("add_obstacle", &Simulator::AddObstacleCB, this);
  reset_map_server_ =
      _nh.advertiseService("reset_map", &Simulator::ResetMapCB, this);
  step_server_ = _nh.advertiseService("step", &Simulator::StepCB, this);

  // Timer
  mainloop_timer_ =
      _nh.createTimer(ros::Duration(0.01), &Simulator::MainLoopCB, this);
}

void Simulator::MainLoopCB(const ros::TimerEvent &event) {
  //

  // send uav pose
  geometry_msgs::TransformStamped _tf_msg;
  _tf_msg.header.frame_id = "map";
  _tf_msg.header.stamp = ros::Time::now();
  _tf_msg.child_frame_id = "base_link";
  _tf_msg.transform.translation.x = state_.pose.position.x;
  _tf_msg.transform.translation.y = state_.pose.position.y;
  _tf_msg.transform.translation.z = state_.pose.position.z;
  _tf_msg.transform.rotation.x = state_.pose.orientation.x;
  _tf_msg.transform.rotation.y = state_.pose.orientation.y;
  _tf_msg.transform.rotation.z = state_.pose.orientation.z;
  _tf_msg.transform.rotation.w = state_.pose.orientation.w;
  broadcaster_.sendTransform(_tf_msg);
  // update local goals & display
  UpdateLocalGoals();
  // update state
  UpdateDistanceAngleInfo();
  // send laser scan & display
  UpdateLaserScan();
}

bool Simulator::UpdateLocalGoals() {
  if (global_path_interpolate_.poses.empty()) {
    return false;
  }
  // initialize marker style
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::SPHERE;
  _mk_msg.pose.orientation.w = 1;
  // delete  obstalces
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.action = visualization_msgs::Marker::DELETE;
  for (int32_t i = 0; i < local_goals_.size(); i++) {
    _mk_msg.id = i;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_local_goals_publisher_.publish(_mk_arr_msg);
  local_goals_.clear();
  // get tracking point idx
  double _cur_linear_velocity =
      sqrt(pow(state_.twist.linear.x, 2.0) + pow(state_.twist.linear.y, 2.0));
  double _lead_distance = lead_distance_factor_ * _cur_linear_velocity;
  _lead_distance = std::min(max_leading_distance_,
                            std::max(_lead_distance, min_leading_distance_));
  std::vector<int32_t> _idxs =
      GetTrackingPointIdx(global_path_interpolate_, state_, _lead_distance, 0.1,
                          num_tracking_point_ + 1, cur_tracking_idx_);
  // update local goals
  for (int32_t i = 0; i < _idxs.size(); i++) {
    local_goals_.push_back(global_path_interpolate_.poses[_idxs[i]].pose.position);
  }
  if (!tracking_reward_include_) local_goals_.erase(local_goals_.begin());
  // visualize local goals
  _mk_arr_msg.markers.clear();
  _mk_msg.color.r = 0.0;
  _mk_msg.color.g = 1.0;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 0.5;
  _mk_msg.scale.x = 0.1;
  _mk_msg.scale.y = 0.1;
  _mk_msg.scale.z = 0.1;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  for (int32_t i = 0; i < local_goals_.size(); i++) {
    _mk_msg.pose.position = local_goals_[i];
    _mk_msg.id = i;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_local_goals_publisher_.publish(_mk_arr_msg);

}
bool Simulator::UpdateDistanceAngleInfo() {
  int32_t _goal_num = local_goals_.size();
  state_.target_distance.clear();
  state_.target_angle.clear();
  for (int32_t i = 0; i < _goal_num; i++) {
    state_.target_distance.push_back(
        std::sqrt(pow(state_.pose.position.x - local_goals_[i].x, 2.0) +
                  pow(state_.pose.position.y - local_goals_[i].y, 2.0)));
    double _angle_uav_target = std::atan2(state_.pose.position.y - local_goals_[i].y,
                                          state_.pose.position.x - local_goals_[i].x);
    double _angle_uav = tf2::getYaw(state_.pose.orientation);
    state_.target_angle.push_back(
        angles::shortest_angular_distance(_angle_uav, _angle_uav_target));
  }
}
bool Simulator::UpdateLaserScan() {
  //
  sensor_msgs::LaserScan _ls_msg;
  _ls_msg.angle_max = angle_max_;
  _ls_msg.angle_min = angle_min_;
  _ls_msg.angle_increment = (angle_max_ - angle_min_) / (num_laser_ - 1);
  _ls_msg.header.frame_id = "laser_scan";
  _ls_msg.header.stamp = ros::Time::now();
  _ls_msg.range_max = range_max_;
  _ls_msg.range_min = range_min_;
  _ls_msg.ranges.clear();
  _ls_msg.intensities.clear();
  _ls_msg.time_increment = 0.01 / num_laser_;
  _ls_msg.scan_time = 0.01;
  //
  double _angle_base = tf2::getYaw(state_.pose.orientation);
  _angle_base += M_PI_2 + angle_min_;
  for (int32_t i = 0; i < num_laser_; i++) {
    // update range
    double _angle = _angle_base + i * _ls_msg.angle_increment;
    geometry_msgs::Point _outpoint;
    _outpoint.x = range_max_;
    _outpoint.y = 0;
    Rotate(_angle, _outpoint);
    Translate(state_.pose.position, _outpoint);
    double _range_distance = DBL_MAX;
    geometry_msgs::Point _cross_point;
    for (int32_t i = 0; i < pos_obs_.size(); i++) {
      if (LineCircleShortestCrossPoint(pos_obs_[i], radius_obs_[i],
                                       state_.pose.position, _outpoint,
                                       _cross_point)) {
        //
        double _range_distance_t = Distance(state_.pose.position, _cross_point);
        if (_range_distance_t < _range_distance) {
          _range_distance = _range_distance_t;
        }
      }
    }
    for (int32_t i = 0; i < pos_obs_extra_.size(); i++) {
      if (LineCircleShortestCrossPoint(pos_obs_extra_[i], radius_obs_extra_[i],
                                       state_.pose.position, _outpoint,
                                       _cross_point)) {
        //
        double _range_distance_t = Distance(state_.pose.position, _cross_point);
        if (_range_distance_t < _range_distance) {
          _range_distance = _range_distance_t;
        }
      }
    }
    if (_range_distance > range_max_) {
      _range_distance = range_max_ - 0.1;
    }
    if (_range_distance < range_min_) {
      _range_distance = range_min_;
    }
    _ls_msg.ranges.push_back(_range_distance);
    // update intensity
    _ls_msg.intensities.push_back(99999);
  }
  state_.scan = _ls_msg;
  // visualize
  laser_scan_publisher_.publish(state_.scan);
  return true;
}
bool Simulator::ResetMapCB(uav_simulator::ResetMap::Request &req,
                         uav_simulator::ResetMap::Response &resp) {
  //
  //  reset uav pose
  ResetUavPose();
  // reset obstacles & display
  ResetObstacles();
  // get global path & display
  while (!UpdateGlobalPath())  {
    ResetObstacles();
    ros::Duration(0.5).sleep();
  }
  // reset extra obstacles & display
  ResetObstaclesExtra();
  // update local goals & display
  UpdateLocalGoals();
  // update uav state
  UpdateDistanceAngleInfo();
  // display start-goal
  DisplayStartGoal();

  ros::Duration(0.5).sleep();

  resp.state = state_;
  resp.state_vector = GetStateVector(state_);
  resp.success = true;

  return true;
}
bool Simulator::ResetObstacles() {
  //
  // initialize marker style
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  // delete  obstalces
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.action = visualization_msgs::Marker::DELETE;
  for (int32_t i = 0; i < pos_obs_.size(); i++) {
    _mk_msg.id = i;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_obs_publisher_.publish(_mk_arr_msg);
  pos_obs_.clear();
  radius_obs_.clear();
  // get obstacle number
  int32_t _num_obs = num_obs_min_ + rand() % (num_obs_max_ - num_obs_min_ + 1);
  // generate new obstacles
  std::uniform_real_distribution<double> _x_distribution(-length_x_ / 2,
                                                         length_x_ / 2);
  std::uniform_real_distribution<double> _y_distribution(-length_y_ / 2,
                                                         length_y_ / 2);
  std::uniform_real_distribution<double> _radius_distribution(radius_obs_min_,
                                                              radius_obs_max_);
  std::default_random_engine _e(time(NULL));
  for (int32_t i = 0; i < _num_obs; i++) {
    // randomize position and size
    geometry_msgs::Point _point;
    _point.z = 0.5;
    double _radius;
    while (true) {
      // obstacle should keep away from safe areas
      _point.x = _x_distribution(_e);
      _point.y = _y_distribution(_e);
      _radius = _radius_distribution(_e);
      double _dist_start = Distance(_point, start_goal_[0]);
      double _dist_goal = Distance(_point, start_goal_[1]);
      // obstacle should keep away from the start-goal line
      // if (std::fabs(_point.y) < target_distance_ / 2 &&
      //     std::fabs(_point.x) < _radius)
      //   continue;
      if (_dist_start > (safe_radius_ + _radius) &&
          _dist_goal > (safe_radius_ + _radius))
        break;
    }
    pos_obs_.push_back(_point);
    radius_obs_.push_back(_radius);
  }
  // visualize obstacles
  _mk_arr_msg.markers.clear();
  _mk_msg.color.r = 1.0;
  _mk_msg.color.g = 1.0;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.z = 1;
  for (int32_t i = 0; i < _num_obs; i++) {
    _mk_msg.pose.position = pos_obs_[i];
    _mk_msg.id = i;
    _mk_msg.scale.x = radius_obs_[i] * 2;
    _mk_msg.scale.y = radius_obs_[i] * 2;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_obs_publisher_.publish(_mk_arr_msg);

  return true;
}
bool Simulator::ResetUavPose() {
  state_.pose.position = start_goal_[0];
  std::uniform_real_distribution<double> _yaw_distribution(-M_PI_2, M_PI_2);
  std::default_random_engine _e(time(NULL));
  double _yaw = _yaw_distribution(_e);
  tf2::Quaternion _qtn;
  _qtn.setRPY(0, 0, _yaw);
  state_.pose.orientation.x = _qtn.x();
  state_.pose.orientation.y = _qtn.y();
  state_.pose.orientation.z = _qtn.z();
  state_.pose.orientation.w = _qtn.w();
  return true;
}
bool Simulator::ResetObstaclesExtra() {
  //
  // initialize marker style
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  // delete  obstalces
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.action = visualization_msgs::Marker::DELETE;
  for (int32_t i = 0; i < pos_obs_extra_.size(); i++) {
    _mk_msg.id = i;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_obs_extra_publisher_.publish(_mk_arr_msg);
  pos_obs_extra_.clear();
  radius_obs_extra_.clear();
// get obstacle number
  int32_t _num_obs = num_obs_min_extra_ +
                     rand() % (num_obs_max_extra_ - num_obs_min_extra_ + 1);
  // generate new obstacles
  std::uniform_real_distribution<double> _vibration_distribution(-vibration_distance_extra_,
                                                         vibration_distance_extra_);
  std::uniform_real_distribution<double> _radius_distribution(radius_obs_min_extra_,
                                                              radius_obs_max_extra_);
  std::default_random_engine _e(time(NULL));
  for (int32_t i = 0; i < _num_obs; i++) {
    geometry_msgs::Point _point;
    _point.z = 0.5;
    double _radius;
    while (true) {
      // select random global path waypoints
      int32_t _idx  = rand() % global_path_interpolate_.poses.size();
      // obstacle should keep away from safe areas
      _point.x = global_path_interpolate_.poses[_idx].pose.position.x +
                 _vibration_distribution(_e);
      _point.y = global_path_interpolate_.poses[_idx].pose.position.y +
                 _vibration_distribution(_e);
      _radius = _radius_distribution(_e);
      double _dist_start = Distance(_point, start_goal_[0]);
      double _dist_goal = Distance(_point, start_goal_[1]);
      // obstacle should keep away from the start-goal line
      // if (std::fabs(_point.y) < target_distance_ / 2 &&
      //     std::fabs(_point.x) < _radius)
      //   continue;
      if (_dist_start > (2*safe_radius_ + _radius) &&
          _dist_goal > (2*safe_radius_ + _radius))
        break;
    }
    pos_obs_extra_.push_back(_point);
    radius_obs_extra_.push_back(_radius);
  }

  // visualize obstacles
  _mk_arr_msg.markers.clear();
  _mk_msg.color.r = 0.0;
  _mk_msg.color.g = 0.0;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.z = 1;
  for (int32_t i = 0; i < _num_obs; i++) {
    _mk_msg.pose.position = pos_obs_extra_[i];
    _mk_msg.id = i;
    _mk_msg.scale.x = radius_obs_extra_[i] * 2;
    _mk_msg.scale.y = radius_obs_extra_[i] * 2;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  visual_obs_extra_publisher_.publish(_mk_arr_msg);

  return true;
}
bool Simulator::UpdateGlobalPath() {
  //
  // delete markers

  // construct the state space we are planning in
  auto _space(std::make_shared<ob::RealVectorStateSpace>(2));
  // set the bounds for the R^2 part of SE(2)
  ob::RealVectorBounds _bounds(2);
  _bounds.setHigh(0, length_x_/2);
  _bounds.setHigh(1, length_y_/2);
  _bounds.setLow(0, -length_x_/2);
  _bounds.setLow(1, -length_y_/2);
  _space->setBounds(_bounds);
  // construct an instance of  space information from this state space
  auto _si(std::make_shared<ob::SpaceInformation>(_space));
  // set state validity checking for this space
  auto IsStateValid = [&](const ob::State *state) -> bool {
    //
    // get current point
    geometry_msgs::Point _cur_point;
    const ob::RealVectorStateSpace::StateType *_state2D =
        state->as<ob::RealVectorStateSpace::StateType>();
    _cur_point.x = (*_state2D)[0];
    _cur_point.y = (*_state2D)[1];
    for (int32_t i = 0; i < pos_obs_.size(); i++) {
      if (Distance(pos_obs_[i], _cur_point) < radius_obs_[i] + 0.5) {
        return false;
      }
    }
    return true;
  };
  _si->setStateValidityChecker(IsStateValid);
  // create a start state
  ob::ScopedState<> _start(_space);
  _start[0] = start_goal_[0].x;
  _start[1] = start_goal_[0].y;
  // create a start state
  ob::ScopedState<> _goal(_space);
  _goal[0] = start_goal_[1].x;
  _goal[1] = start_goal_[1].y;
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
  ob::PlannerStatus _solved = _planner->ob::Planner::solve(0.5);

  if (_solved) {
    og::PathGeometric *_path =
        _pdef->getSolutionPath()->as<og::PathGeometric>();
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    _path->print(std::cout);
    // construct response msg
    global_path_.poses.clear();
    for (int32_t i = 0; i < _path->getStateCount(); i++) {
      const ob::RealVectorStateSpace::StateType *state =
          _path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
      geometry_msgs::PoseStamped _ps_msg;
      _ps_msg.pose.position.x = (*state)[0];
      _ps_msg.pose.position.y = (*state)[1];
      global_path_.poses.push_back(_ps_msg);
    }
    if (Distance(start_goal_[1], global_path_.poses.back().pose.position) > 0.1) {
      return false;
    }
  } else {
    std::cout << "No solution found" << std::endl;
    return false;
  }
  // visualize global path
  if (_solved) {
    global_path_.header.stamp = ros::Time::now();
    visual_path_publisher_.publish(global_path_);
    global_path_interpolate_ = global_path_;
    Interpolate(global_path_interpolate_, 0.1);
    cur_tracking_idx_ = 0;
  }

  return true;
}
bool Simulator::DisplayStartGoal() {
  //
    // initialize marker style
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  // visualize start-goal
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.color.r = 1.0;
  _mk_msg.color.g = 0.0;
  _mk_msg.color.b = 0.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.type = visualization_msgs::Marker::SPHERE;
  _mk_msg.scale.x = 0.2;
  _mk_msg.scale.y = 0.2;
  _mk_msg.scale.z = 0.2;
  _mk_msg.pose.position = start_goal_[0];
  _mk_msg.id = 0;
  _mk_arr_msg.markers.push_back(_mk_msg);
  _mk_msg.pose.position = start_goal_[1];
  _mk_msg.id = 1;
  _mk_arr_msg.markers.push_back(_mk_msg);
  visual_start_goal_publisher_.publish(_mk_arr_msg);

  return true;
}
std::vector<double> Simulator::GetStateVector(const uav_simulator::State &state_msg) {
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
      (_linear_velocity - max_linear_velocity_ / 2) / (max_linear_velocity_ / 2);
  _result.push_back(_linear_velocity);
  double _angle_velocity = state_msg.twist.angular.z / max_angular_velocity_;
  _result.push_back(_angle_velocity);
  // add target info
  for (int32_t i = 0; i < local_goals_.size(); i++) {
    double _target_distance =
        (state_msg.target_distance[i] - 1.5) / 1.5;
    _result.push_back(_target_distance);
    double _target_angle = state_msg.target_angle[i] / M_PI;
  _result.push_back(_target_angle);
  }

  return _result;
}
uav_simulator::Reward Simulator::GetReward(const uav_simulator::State &cur_state,
                                  const uav_simulator::State &next_state,
                                  const vector<geometry_msgs::Point> &local_goals, 
                                  const double step_time, const bool is_arrival,
                                  const bool is_crash,
                                  const int32_t step_count) {
  //
  // distance reward 
  vector<float_t> _distance_rewards;
  vector<double> _allocation_factors(local_goals.size(), 1.0 / local_goals.size());
  if (fabs(distance_reward_allocation_factor_ - 1.0) >
      std::numeric_limits<double>::epsilon()) {
    //
    double _factor_t = 1.0;
    double _corrective_factor =
        (1 - std::pow(distance_reward_allocation_factor_,
                      _allocation_factors.size())) /
        (1 - distance_reward_allocation_factor_);
    for (int32_t i = 0, _n = _allocation_factors.size(); i < _n; i++) {
      _allocation_factors[_n - 1 - i] = _factor_t / _corrective_factor;
      _factor_t *= distance_reward_allocation_factor_;
    }
  }
  double _distance_reward = 0.0;
  for (int32_t i = 0, _n = local_goals.size(); i < _n; i++) {
    double _dist_prev = Distance(cur_state.pose.position, local_goals[i]);
    double _dist_rear = Distance(next_state.pose.position, local_goals[i]);
    double _distance_reward_t = _allocation_factors[i] *
                                (_dist_prev - _dist_rear) * (5 / step_time) *
                                1.6;
    _distance_rewards.push_back(_distance_reward_t);
    _distance_reward += _distance_reward_t;
  }
  // tracking reward
  double _tracking_reward = 0.0;
  if (local_goals.size() != 1) {
    double _tracking_error = next_state.target_distance.front();
   _tracking_reward = -  1.0 * _tracking_error;
  }
  // arrival reward
  double _arrival_reward  = 0.0;
  // double _goal_distance = Distance(next_state.pose.position, start_goal_[1]);
  // _arrival_reward = 5.0 * std::exp(-_goal_distance/0.1);
  if (is_arrival) _arrival_reward = 15;
  // crash reward
  double _crash_reward = 0.0;
  double _min_range = *std::min_element(next_state.scan.ranges.begin(),
                                        next_state.scan.ranges.end()) -
                      crash_limit_;
  _crash_reward = -15.0 * std::exp(-_min_range/0.15);
  // laser reward
  double _laser_reward = 0.0;
  for (int32_t i = 0, _n = next_state.scan.ranges.size(); i < _n; i++) {
    double _range = (next_state.scan.ranges[i] - next_state.scan.range_min) /
                    (next_state.scan.range_max - next_state.scan.range_min);
    _laser_reward += -0.3 * std::exp(-_range/0.1);
  }
  _laser_reward = std::max(_laser_reward, -100.0);
  // step reward
  double _step_reward = -step_count * 0.02;
  // total reward
  double _total_reward = _distance_reward + _tracking_reward + _arrival_reward +
                         _crash_reward + _laser_reward + _step_reward;

  uav_simulator::Reward _reward_msg;
  _reward_msg.distance_rewards = _distance_rewards;
  _reward_msg.distance_reward = _distance_reward;
  _reward_msg.tracking_reward = _tracking_reward;
  _reward_msg.arrive_reward  = _arrival_reward;
  _reward_msg.crash_reward  = _crash_reward;
  _reward_msg.laser_reward = _laser_reward;
  _reward_msg.step_punish_reward = _step_reward;
  _reward_msg.total_reward  = _total_reward;

  return _reward_msg;
}

bool Simulator::StepCB(uav_simulator::Step::Request &req,
                     uav_simulator::Step::Response &resp) {
  //
  uav_simulator::State _cur_state = state_;
  vector<geometry_msgs::Point> _cur_local_goals = local_goals_;
  Intergrator(state_, req.control, req.step_time);
  uav_simulator::State _next_state = state_;
  resp.state = state_;
  // update state
  bool UpdateDistanceAngleInfo();
  // send laser scan
  UpdateLaserScan();
  laser_scan_publisher_.publish(state_.scan);

  resp.state_vector = GetStateVector(state_);
  resp.is_crash = IsCrash(state_);
  resp.is_arrive = IsArrival(state_);
  resp.is_out_range = IsOutRange(state_);
  resp.reward =
      GetReward(_cur_state, _next_state, _cur_local_goals, req.step_time,
                resp.is_arrive, resp.is_crash, req.step_count);
  resp.success = true;
  return true;
}

bool Simulator::AddObstacleCB(uav_simulator::AddObstacle::Request &req,
                   uav_simulator::AddObstacle::Response &resp) {
  //
  visualization_msgs::MarkerArray _mk_arr_msg;
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  _mk_msg.color.r = 0.0;
  _mk_msg.color.g = 0.0;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.x = req.obs_radius * 2;
  _mk_msg.scale.y = req.obs_radius * 2;
  _mk_msg.scale.z = 1.0;
  _mk_msg.pose.position.z = 0.5;
  for (int32_t i = 0; i < req.obs_x.size(); i++) {
    _mk_msg.pose.position.x = req.obs_x[i];
    _mk_msg.pose.position.y = req.obs_y[i];
    _mk_msg.id = pos_obs_extra_.size();
    _mk_arr_msg.markers.push_back(_mk_msg);
      geometry_msgs::Point _point;
    _point.x = req.obs_x[i];
    _point.y = req.obs_y[i];
    _point.z = 0.5;
    pos_obs_extra_.push_back(_point);
    radius_obs_extra_.push_back(req.obs_radius);
  }

  visual_obs_extra_publisher_.publish(_mk_arr_msg);

  resp.success = true;
  return true;
}
void Simulator::RvizGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p) {
  //
  visualization_msgs::MarkerArray _mk_arr_msg;
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  _mk_msg.color.r = 0.0;
  _mk_msg.color.g = 0.0;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.x = 0.25 * 2;
  _mk_msg.scale.y = 0.25 * 2;
  _mk_msg.scale.z = 1.0;
  _mk_msg.pose.position.x = msg_p->pose.position.x;
  _mk_msg.pose.position.y = msg_p->pose.position.y;
  _mk_msg.pose.position.z = 0.5;
  _mk_msg.id = pos_obs_extra_.size();
  _mk_arr_msg.markers.push_back(_mk_msg);
  visual_obs_extra_publisher_.publish(_mk_arr_msg);

  geometry_msgs::Point _point;
  _point.x = msg_p->pose.position.x;
  _point.y = msg_p->pose.position.y;
  _point.z = 0.5;
  pos_obs_extra_.push_back(_point);
  radius_obs_extra_.push_back(0.25);
}
void Simulator::UpdateModel(uav_simulator::State &state,
                            const uav_simulator::Control control,
                            const double duration) {
  //
  double _yaw, _pitch, _roll;
  tf2::getEulerYPR(state.pose.orientation, _yaw, _pitch, _roll);
  // update velicity
  double _alpha = duration / intergrate_dt_ * state_update_factor_;
  double _cur_linear_velocity = std::sqrt(std::pow(state.twist.linear.x, 2.0) +
                                          std::pow(state.twist.linear.y, 2.0));
  _cur_linear_velocity =
      (1 - _alpha) * _cur_linear_velocity + _alpha * control.linear_velocity;
  state.twist.linear.x = _cur_linear_velocity * std::cos(_yaw + M_PI_2);
  state.twist.linear.y = _cur_linear_velocity * std::sin(_yaw + M_PI_2);
  state.twist.angular.z =
      (1 - _alpha) * state.twist.angular.z + _alpha * control.yaw_rate;
  // modify position
  state.pose.position.x += state.twist.linear.x * duration;
  state.pose.position.y += state.twist.linear.y * duration;
  // modify orientation
  _yaw += state.twist.angular.z * duration;
  tf2::Quaternion _qtn;
  _qtn.setRPY(_roll, _pitch, _yaw);
  state.pose.orientation.x = _qtn.x();
  state.pose.orientation.y = _qtn.y();
  state.pose.orientation.z = _qtn.z();
  state.pose.orientation.w = _qtn.w();
}
void Simulator::Intergrator(uav_simulator::State &state,
                            const uav_simulator::Control control,
                            const double duration) {
  //
  double _intergrate_time = intergrate_dt_;
  while (_intergrate_time < duration + std::numeric_limits<double>::epsilon()) {
    UpdateModel(state, control, intergrate_dt_);
    _intergrate_time += intergrate_dt_;
    ros::Duration(intergrate_dt_/accelerate_rate_).sleep();
    ros::spinOnce();
    if (IsCrash(state) || IsArrival(state)) break;
  }
  if (_intergrate_time + std::numeric_limits<double>::epsilon() > duration) {
    UpdateModel(state, control, _intergrate_time - duration);
    ros::Duration(_intergrate_time - duration/accelerate_rate_).sleep();
    ros::spinOnce();
  }
}
bool Simulator::IsCrash(const uav_simulator::State &state) {
  //
  for (int32_t i = 0; i < pos_obs_.size(); i++) {
    if (Distance(pos_obs_[i], state_.pose.position) <
        radius_obs_[i] + crash_limit_) {
      return true;
    }
  }
  for (int32_t i = 0; i < pos_obs_extra_.size(); i++) {
    if (Distance(pos_obs_extra_[i], state_.pose.position) <
        radius_obs_extra_[i] + crash_limit_) {
      return true;
    }
  }
  return false;
}
bool Simulator::IsArrival(const uav_simulator::State &state) {
  //
  return Distance(state_.pose.position, start_goal_[1]) < arrive_limit_;
}
bool Simulator::IsOutRange(const uav_simulator::State &state) {
  //
  double _x = state.pose.position.x;
  double _y = state.pose.position.y;
  return (_x < -length_x_ / 2) || (_x > length_x_ / 2) ||
         (_y < -length_y_ / 2) || (_y > length_y_ / 2);
}


int32_t main(int32_t argc, char *argv[]) {
  srand(time(NULL));
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

  Simulator _ins;
  ros::spin();

  return 0;
}