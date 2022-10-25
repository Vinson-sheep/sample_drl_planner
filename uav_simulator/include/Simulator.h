// #pragma once
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_srvs/Empty.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "uav_simulator/SetUavPose.h"
#include "uav_simulator/Step.h"
#include <vector>
#include "uav_simulator/State.h"
#include "uav_simulator/Control.h"
#include "uav_simulator/ResetMap.h"
#include "tf2/utils.h"
#include <random>

using std::vector;

class Simulator {
 public:
 Simulator();
 ~Simulator();
  //
 private:

  // uav state
  double crash_limit_;
  double arrive_limit_;
  double range_min_;
  double range_max_;
  int32_t num_laser_;
  double uav_radius_;
  double uav_max_linear_velocity_;
  double uav_max_yaw_rate_;
  double uav_max_linear_acc;
  double uav_max_angle_acc;
  double flight_height_;
  uav_simulator::State state_;
  double intergrate_dt_; 

  // grid map
  double target_distance_;
  double safe_radius_;
  double length_x_;
  double length_y_;
  int32_t num_obs_max_;
  int32_t num_obs_min_;
  double radius_obs_max_;
  double radius_obs_min_;
  vector<geometry_msgs::Point> pos_obs_;
  vector<double> radius_obs_;  

  // Publisher & tf
  ros::Publisher grid_map_publisher_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // Servicer
  ros::ServiceServer reset_map_server_;
  ros::ServiceServer set_uav_pose_server_;
  ros::ServiceServer get_state_server_;
  ros::ServiceServer step_server_;

  // Timer
  ros::Timer mainloop_timer_;

  //
  void MainLoopCB(const ros::TimerEvent &event);
  bool ResetMap(uav_simulator::ResetMap::Request &req, uav_simulator::ResetMap::Response &resp);
  bool ResetMapAndDisplay();
  bool SetUAVPose(uav_simulator::SetUavPose::Request &req, uav_simulator::SetUavPose::Response &resp);
  bool Step(uav_simulator::Step::Request &req, uav_simulator::Step::Response &resp);

  // model
  void UpdateModel(uav_simulator::State &state, const uav_simulator::Control control);
  void Intergrator(uav_simulator::State &state, const uav_simulator::Control control, const double duration);
  bool IsCrash(const uav_simulator::State &state);
  bool IsArrival(const uav_simulator::State &state);

  // tool function
  double Distance(const geometry_msgs::Point &lhs,
                  const geometry_msgs::Point &rhs);
  double Distance(const geometry_msgs::Pose &lhs,
                  const geometry_msgs::Pose &rhs);
};

#endif /* SIMULATOR_H */
