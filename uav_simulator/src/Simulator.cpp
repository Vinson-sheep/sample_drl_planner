#include "Simulator.h"
#include "angles/angles.h"

Simulator::Simulator() {
  //
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;

  // uav state
  max_linear_velocity_ = _private_nh.param("max_linear_velocity", 1.0);
  max_angular_velocity_ = _private_nh.param("max_angular_velocity", 1.0);
  crash_limit_ = _private_nh.param("crash_limit", 0.2);
  arrive_limit_ = _private_nh.param("arrive_limit", 0.2);
  angle_max_ = _private_nh.param("angle_max", 2 * M_PI / 3);
  angle_min_ = _private_nh.param("angle_min", -2 * M_PI / 3);
  range_max_ = _private_nh.param("range_max", 3.0);
  range_min_ = _private_nh.param("range_min", 0.15);
  num_laser_ = _private_nh.param("num_laser", 40);
  flight_height_ = _private_nh.param("flight_height", 0.5);
  intergrate_dt_ = _private_nh.param("intergrate_dt", 0.02);
  accelerate_rate_ = _private_nh.param("accelerate_rate", 1.0);
  state_update_factor_ = _private_nh.param("state_update_factor", 0.1);
  state_.pose.orientation.w = 1.0;

  // grid map
  target_distance_ = _private_nh.param("target_distance", 8.0);
  safe_radius_ = _private_nh.param("safe_radius", 0.5);
  length_x_ = _private_nh.param("length_x", 15);
  length_y_ = _private_nh.param("length_y", 15);
  num_obs_max_ = _private_nh.param("num_obs_max", 20);
  num_obs_min_ = _private_nh.param("num_obs_min", 20);
  radius_obs_max_ = _private_nh.param("radius_obs_max", 2.0);
  radius_obs_min_ = _private_nh.param("radius_obs_min", 0.25);
  num_obs_max_extra_ = _private_nh.param("num_obs_max_extra", 5);
  num_obs_min_extra_ = _private_nh.param("num_obs_min_extra", 2);
  radius_obs_max_extra_ = _private_nh.param("radius_obs_max_extra", 0.3);
  radius_obs_min_extra_ = _private_nh.param("radius_obs_min_extra", 0.25);
  vibration_distance_extra_ = _private_nh.param("vibration_distance_extra", 0.5);
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
  lead_distance_factor_ = _private_nh.param("lead_distance_factor", 5.0);
  max_leading_distance_ = _private_nh.param("max_leading_distance", 5.0);
  min_leading_distance_ = _private_nh.param("min_leading_distance", 1.0);
  num_tracking_point_ = _private_nh.param("num_tracking_point", 5);  

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
  reset_map_server_ =
      _nh.advertiseService("reset_map", &Simulator::ResetMap, this);
  step_server_ = _nh.advertiseService("step", &Simulator::Step, this);
  add_obs_server_ =
      _nh.advertiseService("add_obstacle", &Simulator::AddObstacleCB, this);

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
                          num_tracking_point_, cur_tracking_idx_);
  // update local goals
  for (int32_t i = 0; i < _idxs.size(); i++) {
    local_goals_.push_back(global_path_interpolate_.poses[_idxs[i]].pose.position);
  }
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
bool Simulator::ResetMap(uav_simulator::ResetMap::Request &req,
                         uav_simulator::ResetMap::Response &resp) {
  //
//  reset uav pose
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
  // reset obstacles & display
  ResetObstacles();
  // get global path & display
  UpdateGlobalPath();
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
      if (_dist_start > (safe_radius_ + _radius) &&
          _dist_goal > (safe_radius_ + _radius))
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
    std::vector<geometry_msgs::Point> _points;
    for (int32_t i = 0; i < _path->getStateCount(); i++) {
      const ob::RealVectorStateSpace::StateType *state =
          _path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
      geometry_msgs::PoseStamped _ps_msg;
      _ps_msg.pose.position.x = (*state)[0];
      _ps_msg.pose.position.y = (*state)[1];
      global_path_.poses.push_back(_ps_msg);
    }
  } else {
    std::cout << "No solution found" << std::endl;
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
std::vector<double> Simulator::GetStateVector(const uav_simulator::State state_msg) {
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
        (state_msg.target_distance[i] - 2.5) / 2.5;
    _result.push_back(_target_distance);
    double _target_angle = state_msg.target_angle[i] / M_PI;
  _result.push_back(_target_angle);
  }

  return _result;
}
// bool Simulator::ResetMapAndDisplay() {
//   //
//   // initialize marker style
//   visualization_msgs::Marker _mk_msg;
//   _mk_msg.header.frame_id = "map";
//   _mk_msg.header.stamp = ros::Time::now();
//   _mk_msg.type = visualization_msgs::Marker::CYLINDER;
//   _mk_msg.pose.orientation.w = 1;

//   // delete  obstalces
//   visualization_msgs::MarkerArray _mk_arr_msg;
//   _mk_msg.action = visualization_msgs::Marker::DELETE;
//   for (int32_t i = 0; i < pos_obs_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_obs_publisher_.publish(_mk_arr_msg);
//   pos_obs_.clear();
//   radius_obs_.clear();
//   // delete extra obstacles
//   _mk_arr_msg.markers.clear();
//   for (int32_t i = 0; i < pos_obs_extra_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_obs_extra_publisher_.publish(_mk_arr_msg);
//   pos_obs_extra_.clear();
//   radius_obs_extra_.clear();
//   // delete local goals
//   _mk_arr_msg.markers.clear();
//   for (int32_t i = 0; i < local_goals_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_local_goals_publisher_.publish(_mk_arr_msg);
//   local_goals_.clear();
//   // delete start-goal
//   _mk_arr_msg.markers.clear();
//   for (int32_t i = 0; i < start_goal_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_start_goal_publisher_.publish(_mk_arr_msg);
  
//   // get obstacle number
//   int32_t _num_obs = num_obs_min_ + rand() % (num_obs_max_ - num_obs_min_ + 1);
//   // generate new obstacles
//   std::uniform_real_distribution<double> _x_distribution(-length_x_ / 2,
//                                                          length_x_ / 2);
//   std::uniform_real_distribution<double> _y_distribution(-length_y_ / 2,
//                                                          length_y_ / 2);
//   std::uniform_real_distribution<double> _radius_distribution(radius_obs_min_,
//                                                               radius_obs_max_);
//   std::default_random_engine _e(time(NULL));
//   for (int32_t i = 0; i < _num_obs; i++) {
//     // randomize position and size
//     geometry_msgs::Point _point;
//     _point.z = 0.5;
//     double _radius;
//     while (true) {
//       // obstacle should keep away from safe areas
//       _point.x = _x_distribution(_e);
//       _point.y = _y_distribution(_e);
//       _radius = _radius_distribution(_e);
//       double _dist_start = Distance(_point, start_goal_[0]);
//       double _dist_goal = Distance(_point, start_goal_[1]);
//       // obstacle should keep away from the start-goal line
//       if (std::fabs(_point.y) < target_distance_ / 2 &&
//           std::fabs(_point.x) < _radius)
//         continue;
//       if (_dist_start > (safe_radius_ + _radius) &&
//           _dist_goal > (safe_radius_ + _radius))
//         break;
//     }
//     pos_obs_.push_back(_point);
//     radius_obs_.push_back(_radius);
//   }
//   // visualize obstacles
//   _mk_arr_msg.markers.clear();
//   _mk_msg.color.r = 1.0;
//   _mk_msg.color.g = 1.0;
//   _mk_msg.color.b = 1.0;
//   _mk_msg.color.a = 0.9;
//   _mk_msg.action = visualization_msgs::Marker::ADD;
//   _mk_msg.scale.z = 1;
//   for (int32_t i = 0; i < _num_obs; i++) {
//     _mk_msg.pose.position = pos_obs_[i];
//     _mk_msg.id = i;
//     _mk_msg.scale.x = radius_obs_[i] * 2;
//     _mk_msg.scale.y = radius_obs_[i] * 2;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_obs_publisher_.publish(_mk_arr_msg);
//   // visualize start-goal
//   _mk_arr_msg.markers.clear();
//   _mk_msg.color.r = 1.0;
//   _mk_msg.color.g = 0.0;
//   _mk_msg.color.b = 0.0;
//   _mk_msg.color.a = 0.9;
//   _mk_msg.type = visualization_msgs::Marker::SPHERE;
//   _mk_msg.scale.x = 0.2;
//   _mk_msg.scale.y = 0.2;
//   _mk_msg.scale.z = 0.2;
//   _mk_msg.pose.position = start_goal_[0];
//   _mk_msg.id = 0;
//   _mk_arr_msg.markers.push_back(_mk_msg);
//   _mk_msg.pose.position = start_goal_[1];
//   _mk_msg.id = 1;
//   _mk_arr_msg.markers.push_back(_mk_msg);
//   visual_start_goal_publisher_.publish(_mk_arr_msg);
//   // reset uav pose
//   state_.pose.position = start_goal_[0];
//   std::uniform_real_distribution<double> _yaw_distribution(-M_PI_2, M_PI_2);
//   double _yaw = _yaw_distribution(_e);
//   tf2::Quaternion _qtn;
//   _qtn.setRPY(0, 0, _yaw);
//   state_.pose.orientation.x = _qtn.x();
//   state_.pose.orientation.y = _qtn.y();
//   state_.pose.orientation.z = _qtn.z();
//   state_.pose.orientation.w = _qtn.w();
//   // update state
//   UpdateDistanceInfo();
//   // send laser scan
//   UpdateLaserScan();
//   laser_scan_publisher_.publish(state_.scan);
// }


// bool Simulator::SetUAVPose(uav_simulator::SetUavPose::Request &req,
//                            uav_simulator::SetUavPose::Response &resp) {
//   //
//   state_.pose = req.pose;
//   resp.success = true;
//   return true;
// }
bool Simulator::Step(uav_simulator::Step::Request &req,
                     uav_simulator::Step::Response &resp) {
  //
  Intergrator(state_, req.control, req.step_time);
  resp.state = state_;
  // update state
  bool UpdateDistanceAngleInfo();
  // send laser scan
  UpdateLaserScan();
  laser_scan_publisher_.publish(state_.scan);

  resp.is_crash = IsCrash(state_);
  resp.is_arrive = IsArrival(state_);
  resp.success = true;
  return true;
}
// bool Simulator::SetGoals(uav_simulator::SetGoal::Request &req,
//                uav_simulator::SetGoal::Response &resp) {
//   //
//   // initialize marker style
//   visualization_msgs::MarkerArray _mk_arr_msg;
//   visualization_msgs::Marker _mk_msg;
//   _mk_msg.header.frame_id = "map";
//   _mk_msg.header.stamp = ros::Time::now();
//   _mk_msg.type = visualization_msgs::Marker::CYLINDER;
//   _mk_msg.pose.orientation.w = 1;
//   _mk_msg.action = visualization_msgs::Marker::DELETE;
//   // delete local goals
//   _mk_arr_msg.markers.clear();
//   for (int32_t i = 0; i < local_goals_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_local_goals_publisher_.publish(_mk_arr_msg);
//   local_goals_ = req.position;
//   // update state
//   UpdateDistanceInfo();
//   // visualize local goals
//   _mk_arr_msg.markers.clear();
//   _mk_msg.color.r = 1.0;
//   _mk_msg.color.g = 0.0;
//   _mk_msg.color.b = 0.0;
//   _mk_msg.color.a = 0.9;
//   _mk_msg.type = visualization_msgs::Marker::SPHERE;
//   _mk_msg.scale.x = 0.1;
//   _mk_msg.scale.y = 0.1;
//   _mk_msg.scale.z = 0.1;
//   for (int32_t i = 0; i < local_goals_.size(); i++) {
//     _mk_msg.id = i;
//     _mk_arr_msg.markers.push_back(_mk_msg);
//   }
//   visual_local_goals_publisher_.publish(_mk_arr_msg);

//   resp.state = state_;
//   resp.success = true;
//   return true;
// }
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
  return false;
}
bool Simulator::IsArrival(const uav_simulator::State &state) {
  //
  return Distance(state_.pose.position, start_goal_[1]) < arrive_limit_;
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