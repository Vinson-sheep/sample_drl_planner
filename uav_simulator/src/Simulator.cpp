#include "Simulator.h"
#include "angles/angles.h"

Simulator::Simulator() {
  //
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
  flight_height_ = 0.5;
  state_.pose.orientation.w = 1.0;
  intergrate_dt_ = 0.02;
  accelerate_rate_ = 1.0;
  state_update_factor_ = 0.1;

  // grid map
  target_distance_ = 8.0;
  safe_radius_ = 0.25;
  length_x_ = 15;
  length_y_ = 15;
  num_obs_max_ = 20;
  num_obs_min_ = 20;
  radius_obs_max_ = 2.0;
  radius_obs_min_ = 0.25;
  start_goal_.resize(2);
  start_goal_[0].x = 0;
  start_goal_[0].y = -target_distance_ / 2;
  start_goal_[0].z = flight_height_;
  start_goal_[1].x = 0;
  start_goal_[1].y = target_distance_ / 2;
  start_goal_[1].z = flight_height_;

  // Subscriber
  // rviz_goal_sub_ = _nh.subscribe<geometry_msgs::PoseStamped>(
  //     "/move_base_simple/goal", 1, &Simulator::RvizGoalCB, this);

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

  // // Servicer
  // reset_map_server_ =
  //     _nh.advertiseService("reset_map", &Simulator::ResetMap, this);
  // set_uav_pose_server_ =
  //     _nh.advertiseService("set_uav_pose", &Simulator::SetUAVPose, this);
  // step_server_ = _nh.advertiseService("step", &Simulator::Step, this);
  // get_obs_server_ = _nh.advertiseService("get_obstacle", &Simulator::GetObstacle, this);
  // set_goals_server_ = _nh.advertiseService("set_goals", &Simulator::SetGoals, this);
  // add_obs_server_ = _nh.advertiseService("add_obstacle", &Simulator::AddObstacleCB, this);

  // // Timer
  // mainloop_timer_ =
  //     _nh.createTimer(ros::Duration(0.01), &Simulator::MainLoopCB, this);
}

// void Simulator::MainLoopCB(const ros::TimerEvent &event) {
//   //

//   // send uav pose
//   geometry_msgs::TransformStamped _tf_msg;
//   _tf_msg.header.frame_id = "map";
//   _tf_msg.header.stamp = ros::Time::now();
//   _tf_msg.child_frame_id = "base_link";
//   _tf_msg.transform.translation.x = state_.pose.position.x;
//   _tf_msg.transform.translation.y = state_.pose.position.y;
//   _tf_msg.transform.translation.z = state_.pose.position.z;
//   _tf_msg.transform.rotation.x = state_.pose.orientation.x;
//   _tf_msg.transform.rotation.y = state_.pose.orientation.y;
//   _tf_msg.transform.rotation.z = state_.pose.orientation.z;
//   _tf_msg.transform.rotation.w = state_.pose.orientation.w;
//   broadcaster_.sendTransform(_tf_msg);

//   // update state
//   UpdateDistanceInfo();
//   // send laser scan
//   UpdateLaserScan();
//   laser_scan_publisher_.publish(state_.scan);
// }
// void Simulator::UpdateDistanceInfo() {
//   int32_t _goal_num = local_goals_.size();
//   state_.target_distance.clear();
//   state_.target_distance.clear();
//   for (int32_t i = 0; i < _goal_num; i++) {
//     state_.target_distance.push_back(
//         std::sqrt(pow(state_.pose.position.x - local_goals_[i].x, 2.0) +
//                   pow(state_.pose.position.y - local_goals_[i].y, 2.0)));
//     double _angle_uav_target = std::atan2(state_.pose.position.y - local_goals_[i].y,
//                                           state_.pose.position.x - local_goals_[i].x);
//     double _angle_uav = tf2::getYaw(state_.pose.orientation);
//     state_.target_angle.push_back(
//         angles::shortest_angular_distance(_angle_uav, _angle_uav_target));
//   }
// }
// bool Simulator::ResetMap(uav_simulator::ResetMap::Request &req,
//                          uav_simulator::ResetMap::Response &resp) {
//   //
//   // update parameters
//   crash_limit_ = req.param.crash_limit;
//   arrive_limit_ = req.param.arrive_limit;
//   angle_max_ = req.sparam.angle_max;
//   angle_min_ = req.sparam.angle_min;
//   range_max_ = req.sparam.range_max;
//   range_min_ = req.sparam.range_min;
//   num_laser_ = req.sparam.num_laser;
//   intergrate_dt_ = req.param.intergrate_dt;
//   accelerate_rate_ = req.param.accelerate_rate;
//   target_distance_ = req.param.target_distance;
//   safe_radius_ = req.param.safe_radius;
//   length_x_ = req.param.length_x;
//   length_y_ = req.param.length_y;
//   num_obs_max_ = req.param.num_obs_max;
//   num_obs_min_ = req.param.num_obs_min;
//   radius_obs_max_ = req.param.radius_obs_max;
//   radius_obs_min_ = req.param.radius_obs_min;
//   // update start and goal position
//   start_goal_[0].x = 0;
//   start_goal_[0].y = -target_distance_ / 2;
//   start_goal_[0].z = flight_height_;
//   start_goal_[1].x = 0;
//   start_goal_[1].y = target_distance_ / 2;
//   start_goal_[1].z = flight_height_;

//   ResetMapAndDisplay();

//   ros::Duration(0.5).sleep();

//   resp.state = state_;
//   resp.success = true;

//   return true;
// }

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

// bool Simulator::UpdateLaserScan() {
//   //
//   sensor_msgs::LaserScan _ls_msg;
//   _ls_msg.angle_max = angle_max_;
//   _ls_msg.angle_min = angle_min_;
//   _ls_msg.angle_increment = (angle_max_ - angle_min_) / (num_laser_ - 1);
//   _ls_msg.header.frame_id = "laser_scan";
//   _ls_msg.header.stamp = ros::Time::now();
//   _ls_msg.range_max = range_max_;
//   _ls_msg.range_min = range_min_;
//   _ls_msg.ranges.clear();
//   _ls_msg.intensities.clear();
//   _ls_msg.time_increment = 0.01 / num_laser_;
//   _ls_msg.scan_time = 0.01;
//   //
//   double _angle_base = tf2::getYaw(state_.pose.orientation);
//   _angle_base += M_PI_2 + angle_min_;
//   for (int32_t i = 0; i < num_laser_; i++) {
//     // update range
//     double _angle = _angle_base + i * _ls_msg.angle_increment;
//     geometry_msgs::Point _outpoint;
//     _outpoint.x = range_max_;
//     _outpoint.y = 0;
//     Rotate(_angle, _outpoint);
//     Translate(state_.pose.position, _outpoint);
//     double _range_distance = DBL_MAX;
//     geometry_msgs::Point _cross_point;
//     for (int32_t i = 0; i < pos_obs_.size(); i++) {
//       if (LineCircleShortestCrossPoint(pos_obs_[i], radius_obs_[i],
//                                        state_.pose.position, _outpoint,
//                                        _cross_point)) {
//         //
//         double _range_distance_t = Distance(state_.pose.position, _cross_point);
//         if (_range_distance_t < _range_distance) {
//           _range_distance = _range_distance_t;
//         }
//       }
//     }
//     for (int32_t i = 0; i < pos_obs_extra_.size(); i++) {
//       if (LineCircleShortestCrossPoint(pos_obs_extra_[i], radius_obs_extra_[i],
//                                        state_.pose.position, _outpoint,
//                                        _cross_point)) {
//         //
//         double _range_distance_t = Distance(state_.pose.position, _cross_point);
//         if (_range_distance_t < _range_distance) {
//           _range_distance = _range_distance_t;
//         }
//       }
//     }
//     if (_range_distance > range_max_) {
//       _range_distance = range_max_ - 0.1;
//     }
//     if (_range_distance < range_min_) {
//       _range_distance = range_min_;
//     }
//     _ls_msg.ranges.push_back(_range_distance);
//     // update intensity
//     _ls_msg.intensities.push_back(99999);
//   }
//   state_.scan = _ls_msg;
//   return true;
// }
// bool Simulator::SetUAVPose(uav_simulator::SetUavPose::Request &req,
//                            uav_simulator::SetUavPose::Response &resp) {
//   //
//   state_.pose = req.pose;
//   resp.success = true;
//   return true;
// }
// bool Simulator::Step(uav_simulator::Step::Request &req,
//                      uav_simulator::Step::Response &resp) {
//   //
//   Intergrator(state_, req.control, req.step_time);
//   resp.state = state_;
//   // update state
//   UpdateDistanceInfo();
//   // send laser scan
//   UpdateLaserScan();
//   laser_scan_publisher_.publish(state_.scan);

//   resp.is_crash = IsCrash(state_);
//   resp.is_arrive = IsArrival(state_);
//   resp.success = true;
//   return true;
// }
// bool Simulator::GetObstacle(uav_simulator::GetObstacle::Request &req,
//                             uav_simulator::GetObstacle::Response &resp) {
//   //
//   if (pos_obs_.empty()) {
//     resp.success = false;
//   }
//   else {
//     resp.data.obstacles_position = pos_obs_;
//     resp.data.obstacles_radius = radius_obs_;
//     resp.success = true;
//   }
//   return true;
// }
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
// bool Simulator::AddObstacleCB(uav_simulator::AddObstacle::Request &req,
//                    uav_simulator::AddObstacle::Response &resp) {
//   //
//   visualization_msgs::MarkerArray _mk_arr_msg;
//   visualization_msgs::Marker _mk_msg;
//   _mk_msg.header.frame_id = "map";
//   _mk_msg.header.stamp = ros::Time::now();
//   _mk_msg.type = visualization_msgs::Marker::CYLINDER;
//   _mk_msg.pose.orientation.w = 1;
//   _mk_msg.color.r = 0.0;
//   _mk_msg.color.g = 0.0;
//   _mk_msg.color.b = 1.0;
//   _mk_msg.color.a = 0.9;
//   _mk_msg.action = visualization_msgs::Marker::ADD;
//   _mk_msg.scale.x = req.obs_radius * 2;
//   _mk_msg.scale.y = req.obs_radius * 2;
//   _mk_msg.scale.z = 1.0;
//   _mk_msg.pose.position.z = 0.5;
//   for (int32_t i = 0; i < req.obs_x.size(); i++) {
//     _mk_msg.pose.position.x = req.obs_x[i];
//     _mk_msg.pose.position.y = req.obs_y[i];
//     _mk_msg.id = pos_obs_extra_.size();
//     _mk_arr_msg.markers.push_back(_mk_msg);
//       geometry_msgs::Point _point;
//     _point.x = req.obs_x[i];
//     _point.y = req.obs_y[i];
//     _point.z = 0.5;
//     pos_obs_extra_.push_back(_point);
//     radius_obs_extra_.push_back(req.obs_radius);
//   }

//   visual_obs_extra_publisher_.publish(_mk_arr_msg);

//   resp.success = true;
//   return true;
// }
// void Simulator::RvizGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p) {
//   //
//   visualization_msgs::MarkerArray _mk_arr_msg;
//   visualization_msgs::Marker _mk_msg;
//   _mk_msg.header.frame_id = "map";
//   _mk_msg.header.stamp = ros::Time::now();
//   _mk_msg.type = visualization_msgs::Marker::CYLINDER;
//   _mk_msg.pose.orientation.w = 1;
//   _mk_msg.color.r = 0.0;
//   _mk_msg.color.g = 0.0;
//   _mk_msg.color.b = 1.0;
//   _mk_msg.color.a = 0.9;
//   _mk_msg.action = visualization_msgs::Marker::ADD;
//   _mk_msg.scale.x = 0.25 * 2;
//   _mk_msg.scale.y = 0.25 * 2;
//   _mk_msg.scale.z = 1.0;
//   _mk_msg.pose.position.x = msg_p->pose.position.x;
//   _mk_msg.pose.position.y = msg_p->pose.position.y;
//   _mk_msg.pose.position.z = 0.5;
//   _mk_msg.id = pos_obs_extra_.size();
//   _mk_arr_msg.markers.push_back(_mk_msg);
//   visual_obs_extra_publisher_.publish(_mk_arr_msg);

//   geometry_msgs::Point _point;
//   _point.x = msg_p->pose.position.x;
//   _point.y = msg_p->pose.position.y;
//   _point.z = 0.5;
//   pos_obs_extra_.push_back(_point);
//   radius_obs_extra_.push_back(0.25);
// }
// void Simulator::UpdateModel(uav_simulator::State &state,
//                             const uav_simulator::Control control,
//                             const double duration) {
//   //
//   double _yaw, _pitch, _roll;
//   tf2::getEulerYPR(state.pose.orientation, _yaw, _pitch, _roll);
//   // update velicity
//   double _alpha = duration / intergrate_dt_ * state_update_factor_;
//   double _cur_linear_velocity = std::sqrt(std::pow(state.twist.linear.x, 2.0) +
//                                           std::pow(state.twist.linear.y, 2.0));
//   _cur_linear_velocity =
//       (1 - _alpha) * _cur_linear_velocity + _alpha * control.linear_velocity;
//   state.twist.linear.x = _cur_linear_velocity * std::cos(_yaw + M_PI_2);
//   state.twist.linear.y = _cur_linear_velocity * std::sin(_yaw + M_PI_2);
//   state.twist.angular.z =
//       (1 - _alpha) * state.twist.angular.z + _alpha * control.yaw_rate;
//   // modify position
//   state.pose.position.x += state.twist.linear.x * duration;
//   state.pose.position.y += state.twist.linear.y * duration;
//   // modify orientation
//   _yaw += state.twist.angular.z * duration;
//   tf2::Quaternion _qtn;
//   _qtn.setRPY(_roll, _pitch, _yaw);
//   state.pose.orientation.x = _qtn.x();
//   state.pose.orientation.y = _qtn.y();
//   state.pose.orientation.z = _qtn.z();
//   state.pose.orientation.w = _qtn.w();
// }
// void Simulator::Intergrator(uav_simulator::State &state,
//                             const uav_simulator::Control control,
//                             const double duration) {
//   //
//   double _intergrate_time = intergrate_dt_;
//   while (_intergrate_time < duration + std::numeric_limits<double>::epsilon()) {
//     UpdateModel(state, control, intergrate_dt_);
//     _intergrate_time += intergrate_dt_;
//     ros::Duration(intergrate_dt_/accelerate_rate_).sleep();
//     ros::spinOnce();
//     if (IsCrash(state) || IsArrival(state)) break;
//   }
//   if (_intergrate_time + std::numeric_limits<double>::epsilon() > duration) {
//     UpdateModel(state, control, _intergrate_time - duration);
//     ros::Duration(_intergrate_time - duration/accelerate_rate_).sleep();
//     ros::spinOnce();
//   }
// }
// bool Simulator::IsCrash(const uav_simulator::State &state) {
//   //
//   for (int32_t i = 0; i < pos_obs_.size(); i++) {
//     if (Distance(pos_obs_[i], state_.pose.position) <
//         radius_obs_[i] + crash_limit_) {
//       return true;
//     }
//   }
//   return false;
// }
// bool Simulator::IsArrival(const uav_simulator::State &state) {
//   //
//   return Distance(state_.pose.position, start_goal_[1]) < arrive_limit_;
// }



int32_t main(int32_t argc, char *argv[]) {
  srand(time(NULL));
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

  Simulator _ins;
  ros::spin();

  return 0;
}