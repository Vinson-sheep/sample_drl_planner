#include "Simulator.h"

Simulator::Simulator() {
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;

  // uav state
  crash_limit_ = 0.2;
  arrive_limit_ = 0.2;
  angle_max_ = 2 * M_PI / 3;
  angle_min_ = -2 * M_PI / 3;
  range_max_ = 3.0;
  range_min_ = 0.15;
  num_laser_ = 40;
  uav_radius_ = 0.1;
  uav_max_linear_velocity_ = 1.0;
  uav_max_yaw_rate_ = M_PI_2;
  flight_height_ = 0.5;
  state_.pose.orientation.w = 1.0;
  intergrate_dt_ = 0.02;
  goal_id_ = -1;

  // grid map
  target_distance_ = 8.0;
  safe_radius_ = 0.25;
  length_x_ = 15;
  length_y_ = 15;
  num_obs_max_ = 20; 
  num_obs_min_ = 20;
  radius_obs_max_ = 2.0;
  radius_obs_min_ = 0.25;

  // Publisher
  grid_map_publisher_ =
      _nh.advertise<visualization_msgs::MarkerArray>("grid_map", 10);
  laser_scan_publisher_ =
      _nh.advertise<sensor_msgs::LaserScan>("laser_scan", 10);

  // Servicer
  reset_map_server_ =
      _nh.advertiseService("reset_map", &Simulator::ResetMap, this);
  set_uav_pose_server_ =
      _nh.advertiseService("set_uav_pose", &Simulator::SetUAVPose, this);
  step_server_ = _nh.advertiseService("step", &Simulator::Step, this);

  // Timer
  mainloop_timer_ =
      _nh.createTimer(ros::Duration(0.01), &Simulator::MainLoopCB, this);
}

Simulator::~Simulator() {}

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

  // send laser scan
  UpdateLaserScan();
  laser_scan_publisher_.publish(state_.scan);
}

bool Simulator::ResetMap(uav_simulator::ResetMap::Request &req,
                         uav_simulator::ResetMap::Response &resp) {
  //
  // update parameters
  target_distance_ = req.param.target_distance;
  safe_radius_ = req.param.safe_radius;
  length_x_ = req.param.length_x;
  length_y_ = req.param.length_y;
  num_obs_max_ = req.param.num_obs_max;
  num_obs_min_ = req.param.num_obs_min;
  radius_obs_max_ = req.param.radius_obs_max;
  radius_obs_min_ = req.param.radius_obs_min;
  // update start and goal position
  goal_.x = 0;
  goal_.y = target_distance_ / 2;
  goal_.z = flight_height_;
  start_.x = 0;
  start_.y = -target_distance_ / 2;
  start_.z = flight_height_;

  ResetMapAndDisplay();

  resp.success = true;

  return true;
}

bool Simulator::ResetMapAndDisplay() {
  //
  // initialize marker style
  visualization_msgs::Marker _mk_msg;
  _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CYLINDER;
  _mk_msg.pose.orientation.w = 1;
  // get obstacle number
  int32_t _num_obs = num_obs_min_ + rand() % (num_obs_max_ - num_obs_min_ + 1);
  // delete original obstalces
  visualization_msgs::MarkerArray _mk_arr_msg;
  _mk_msg.action = visualization_msgs::Marker::DELETE;
  for (int32_t i = 0; i < pos_obs_.size(); i++) {
    _mk_msg.id = i;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  if (goal_id_ != -1) {
    _mk_msg.id = goal_id_;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  grid_map_publisher_.publish(_mk_arr_msg);
  _mk_arr_msg.markers.clear();
  pos_obs_.clear();
  radius_obs_.clear();
  goal_id_ = -1;
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
      double _dist_start = Distance(_point, start_);
      double _dist_goal = Distance(_point, goal_);
      if (_dist_start > (safe_radius_ + _radius) &&
          _dist_goal > (safe_radius_ + _radius))
        break;
    }
    pos_obs_.push_back(_point);
    radius_obs_.push_back(_radius);
  }
  // visualize obstacles
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
  // visualize goal position
  _mk_msg.color.r = 1.0;
  _mk_msg.color.g = 0.0;
  _mk_msg.color.b = 0.0;
  _mk_msg.color.a = 0.9;
  _mk_msg.type = visualization_msgs::Marker::SPHERE;
  _mk_msg.scale.x = 0.2;
  _mk_msg.scale.y = 0.2;
  _mk_msg.scale.z = 0.2;
  _mk_msg.pose.position = goal_;
  _mk_msg.id = _num_obs + 1;
  goal_id_ = _num_obs + 1;
  _mk_arr_msg.markers.push_back(_mk_msg);
  // reset uav pose
  state_.pose.position = start_;
  std::uniform_real_distribution<double> _yaw_distribution(-M_PI_2, M_PI_2);
  double _yaw = _yaw_distribution(_e);
  tf2::Quaternion _qtn;
  _qtn.setRPY(0, 0, _yaw);
  state_.pose.orientation.x = _qtn.x();
  state_.pose.orientation.y = _qtn.y();
  state_.pose.orientation.z = _qtn.z();
  state_.pose.orientation.w = _qtn.w();
  // publish
  grid_map_publisher_.publish(_mk_arr_msg);
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
  return true;
}
bool Simulator::SetUAVPose(uav_simulator::SetUavPose::Request &req,
                           uav_simulator::SetUavPose::Response &resp) {
  //
  state_.pose = req.pose;
  resp.success = true;
  return true;
}
bool Simulator::Step(uav_simulator::Step::Request &req,
                     uav_simulator::Step::Response &resp) {
  //
  Intergrator(state_, req.control, req.step_time);
  resp.state = state_;
  resp.is_crash = IsCrash(state_);
  resp.is_arrive = IsArrival(state_);
  resp.success = true;
  return true;
}
void Simulator::UpdateModel(uav_simulator::State &state,
                            const uav_simulator::Control control) {
  //
  double _yaw, _pitch, _roll;
  tf2::getEulerYPR(state.pose.orientation, _yaw, _pitch, _roll);
  // modify position
  double _dx = control.linear_velocity * cos(_yaw + M_PI_2);
  double _dy = control.linear_velocity * sin(_yaw + M_PI_2);
  state.pose.position.x += _dx;
  state.pose.position.y += _dy;
  // modify orientation
  _yaw += control.yaw_rate;
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
  uav_simulator::Control _sub_ctr;
  double _intergrate_time = intergrate_dt_;
  while (_intergrate_time < duration + std::numeric_limits<double>::epsilon()) {
    _sub_ctr.linear_velocity = intergrate_dt_ * control.linear_velocity;
    _sub_ctr.yaw_rate = intergrate_dt_ * control.yaw_rate;
    UpdateModel(state, _sub_ctr);
    _intergrate_time += intergrate_dt_;
    ros::Duration(intergrate_dt_).sleep();
    ros::spinOnce();
  }
  if (_intergrate_time + std::numeric_limits<double>::epsilon() > duration) {
    _sub_ctr.linear_velocity =
        (_intergrate_time - duration) * control.linear_velocity;
    _sub_ctr.yaw_rate = (_intergrate_time - duration) * control.yaw_rate;
    UpdateModel(state, _sub_ctr);
    ros::Duration(_intergrate_time - duration).sleep();
    ros::spinOnce();
  }
}
bool Simulator::IsCrash(const uav_simulator::State &state) {
  //
  for (int32_t i = 0; i < pos_obs_.size(); i++) {
    if (Distance(pos_obs_[i], state_.pose.position) < radius_obs_[i] + crash_limit_) {
      return true;
    }
  }
  return false;
}
bool Simulator::IsArrival(const uav_simulator::State &state) {
  //
  return Distance(state_.pose.position, goal_) < arrive_limit_;
}

// tool function
double Simulator::Distance(const geometry_msgs::Point &lhs,
                           const geometry_msgs::Point &rhs) {
  //
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
              (lhs.y - rhs.y) * (lhs.y - rhs.y));
}
double Simulator::Distance(const geometry_msgs::Pose &lhs,
                           const geometry_msgs::Pose &rhs) {
  //
  return Distance(lhs.position, rhs.position);
}
bool Simulator::LineCircleShortestCrossPoint(
    const geometry_msgs::Point &center, const double radius,
    geometry_msgs::Point &cur_pos, const geometry_msgs::Point &next_pos,
    geometry_msgs::Point &cross_point) {
  //
  double _line_k, _line_b;
  LineParam(cur_pos, next_pos, _line_k, _line_b);
  double _dist = PointLineDistance(center, _line_k, _line_b);
  //
  if (_dist > radius + std::numeric_limits<double>::epsilon()) {
    //
    return false;
  } else {
    //
    double _A = 1 + _line_k * _line_k;
    double _B = 2 * _line_k * (_line_b - center.y) - 2 * center.x;
    double _C = pow(center.x, 2) + pow(_line_b - center.y, 2) - pow(radius, 2);
    if (fabs(_dist - radius) < std::numeric_limits<double>::epsilon()) {
      cross_point.x = -_B / (2 * _A);
      cross_point.y = _line_k * cross_point.x + _line_b;
    } else {
      double _x1, _x2, _y1, _y2;
      double _D = sqrt(_B * _B - 4 * _A * _C);
      _x1 = (-_B + _D) / (2 * _A);
      _y1 = _line_k * _x1 + _line_b;
      _x2 = (-_B - _D) / (2 * _A);
      _y2 = _line_k * _x2 + _line_b;
      geometry_msgs::Point _cross_point_1;
      _cross_point_1.x = _x1;
      _cross_point_1.y = _y1;
      geometry_msgs::Point _cross_point_2;
      _cross_point_2.x = _x2;
      _cross_point_2.y = _y2;
      double _dist_1 = Distance(_cross_point_1, cur_pos);
      double _dist_2 = Distance(_cross_point_2, cur_pos);
      if (_dist_1 < _dist_2) {
        cross_point.x = _x1;
        cross_point.y = _y1;
      } else {
        cross_point.x = _x2;
        cross_point.y = _y2;
      }
    }
    // judge if cross point is in the right direction
    if (!HomoDirect(cur_pos, cross_point, cur_pos, next_pos)) {
      return false;
    }
  }
  return true;
}
double Simulator::PointLineDistance(const geometry_msgs::Point &point,
                                    const double line_k, const double line_b) {
  //
  return fabs(line_k * point.x - point.y + line_b) /
         sqrt(1.0 + line_k * line_k);
}
bool Simulator::Translate(const geometry_msgs::Point &tr_point,
                          geometry_msgs::Point &point) {
  //
  point.x += tr_point.x;
  point.y += tr_point.y;
  return true;
}
bool Simulator::Rotate(const double theta, geometry_msgs::Point &point) {
  //
  double _dist = sqrt(point.x * point.x + point.y + point.y);
  double _theta_ori = atan2(point.y, point.x);
  _theta_ori += theta;
  point.x = _dist * cos(_theta_ori);
  point.y = _dist * sin(_theta_ori);
  return true;
}

bool Simulator::LineParam(const geometry_msgs::Point &point_a,
                          const geometry_msgs::Point &point_b, double &line_k,
                          double &line_b) {
  //
  double _dx = point_a.x - point_b.x;
  double _dy = point_a.y - point_b.y;
  if (fabs(_dx) < 10e-7) {  // avoid zero divide error
    line_k = 10e17;
  } else {
    line_k = std::max(-10e17, std::min(10e17, _dy / _dx));
  }
  line_b = point_a.y - line_k * point_a.x;
  return true;
}

bool Simulator::HomoDirect(const geometry_msgs::Point &line_a_1,
                           const geometry_msgs::Point &line_a_2,
                           const geometry_msgs::Point &line_b_1,
                           const geometry_msgs::Point &line_b_2) {
  //
  double _dx_a = line_a_1.x - line_a_2.x;
  double _dy_a = line_a_1.y - line_a_2.y;
  double _dx_b = line_b_1.x - line_b_2.x;
  double _dy_b = line_b_1.y - line_b_2.y;
  return fabs(std::atan2(_dy_a, _dx_a) - std::atan2(_dy_b, _dx_b)) < 10e-5;
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