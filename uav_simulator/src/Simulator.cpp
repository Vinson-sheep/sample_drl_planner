#include "Simulator.h"

Simulator::Simulator() {
    
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    // uav state
    crash_limit_ = 0.2;
    arrive_limit_ = 0.2;
    range_max_  = 5.0;
    range_min_ = 0.15;
    num_laser_ = 9;
    uav_radius_ = 0.1;
    uav_max_linear_velocity_ = 1.0;
    uav_max_yaw_rate_ = M_PI_2;
    flight_height_ = 0.5;
    state_.pose.orientation.w = 1.0;
    intergrate_dt_ = 0.01;
    // Publisher
    grid_map_publisher_ = _nh.advertise<visualization_msgs::MarkerArray>("grid_map", 10);
    
    // Servicer
    reset_map_server_ = _nh.advertiseService("reset_map", &Simulator::ResetMap, this);
    set_uav_pose_server_ = _nh.advertiseService("set_uav_pose", &Simulator::SetUAVPose, this);
    step_server_ = _nh.advertiseService("step", &Simulator::Step, this);

    // Timer
    mainloop_timer_ = _nh.createTimer(ros::Duration(0.02), &Simulator::MainLoopCB, this);

}

Simulator::~Simulator() {

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

}

bool Simulator::ResetMap(uav_simulator::ResetMap::Request &req, uav_simulator::ResetMap::Response &resp) {
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
  _goal.x = 0;
  _goal.y = target_distance_/2;
  _goal.z = flight_height_;
  _start.x = 0;
  _start.y = -target_distance_/2;
  _start.z = flight_height_;

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
  int32_t _num_obs = num_obs_min_ + rand() % (num_obs_max_ -num_obs_min_ + 1);
  // delete original obstalces
  visualization_msgs::MarkerArray _mk_arr_msg;
  for (int32_t i=0; i < pos_obs_.size() + 1; i++) { // +1: delele target 
    _mk_msg.id = i;
    _mk_msg.action = visualization_msgs::Marker::DELETE;
    _mk_arr_msg.markers.push_back(_mk_msg);
  }
  grid_map_publisher_.publish(_mk_arr_msg);
  _mk_arr_msg.markers.clear();
  pos_obs_.clear();
  radius_obs_.clear();
  ros::spinOnce();
  // generate new obstacles
  std::uniform_real_distribution<double> _x_distribution(-length_x_/2, length_x_/2);
  std::uniform_real_distribution<double> _y_distribution(-length_y_/2, length_y_/2);
  std::uniform_real_distribution<double> _radius_distribution(radius_obs_min_, radius_obs_max_);
  std::default_random_engine _e(time(NULL));
  for (int32_t i=0; i < _num_obs; i++) {
    // randomize position and size
    geometry_msgs::Point _point;
    _point.z = 0.5;
    double _radius;
    while (true) { 
      // obstacle should keep away from safe areas
      _point.x = _x_distribution(_e);
      _point.y = _y_distribution(_e);
      _radius = _radius_distribution(_e);
      double _dist_start = Distance(_point, _start);
      double _dist_goal = Distance(_point, _goal);
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
  _mk_msg.color.a= 0.9;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.z = 1;
  for (int32_t i=0; i < _num_obs; i++) {
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
  _mk_msg.color.a= 0.9;
  _mk_msg.type = visualization_msgs::Marker::SPHERE;
  _mk_msg.scale.x = 0.2;
  _mk_msg.scale.y = 0.2;
  _mk_msg.scale.z = 0.2;
  _mk_msg.pose.position = _goal;
  _mk_arr_msg.markers.push_back(_mk_msg);
  // reset uav pose
  state_.pose.position = _start;
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

bool Simulator::SetUAVPose(uav_simulator::SetUavPose::Request &req, uav_simulator::SetUavPose::Response &resp) {
  //
  state_.pose = req.pose;
  resp.success = true;
  return true;
}
bool Simulator::Step(uav_simulator::Step::Request &req, uav_simulator::Step::Response &resp) {
  //
  Intergrator(state_, req.control, req.step_time);
  resp.state = state_;
  resp.is_crash = IsCrash(state_);
  resp.is_arrive = IsArrival(state_);
  resp.success = true;
  return true;
}
void Simulator::UpdateModel(uav_simulator::State &state, const uav_simulator::Control control) {
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
void Simulator::Intergrator(uav_simulator::State &state, const uav_simulator::Control control, const double duration) {
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
    _sub_ctr.linear_velocity = (_intergrate_time - duration) * control.linear_velocity;
    _sub_ctr.yaw_rate = (_intergrate_time - duration) * control.yaw_rate;
    UpdateModel(state, _sub_ctr);
    ros::Duration(_intergrate_time - duration).sleep();
    ros::spinOnce();
  }
}
bool Simulator::IsCrash(const uav_simulator::State &state) {
  //
  return false;
}
bool Simulator::IsArrival(const uav_simulator::State &state) {
  // 
  return false;
}

// tool function
double Simulator::Distance(const geometry_msgs::Point &lhs,
                  const geometry_msgs::Point &rhs) {
  //
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y));
}
double Simulator::Distance(const geometry_msgs::Pose &lhs,
                           const geometry_msgs::Pose &rhs) {
  //
  return Distance(lhs.position, rhs.position);
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