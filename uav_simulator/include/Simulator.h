// #pragma once
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <random>
#include <vector>
#include "common.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "uav_simulator/AddObstacle.h"
#include "uav_simulator/Control.h"
#include "uav_simulator/ResetMap.h"
#include "uav_simulator/State.h"
#include "uav_simulator/Step.h"
#include "uav_simulator/Reward.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

using std::vector;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class Simulator {
 public:
  Simulator();
  ~Simulator(){};
  //
 private:
  // uav state
  double max_linear_velocity_;
  double max_angular_velocity_;
  double crash_limit_;
  double arrive_limit_;
  double angle_max_;
  double angle_min_;
  double range_max_;
  double range_min_;
  int32_t num_laser_;
  double flight_height_;
  uav_simulator::State state_;
  double intergrate_dt_;
  double accelerate_rate_;
  double state_update_factor_;

  // grid map
  double target_distance_;
  double safe_radius_;
  double length_x_;
  double length_y_;
  int32_t num_obs_max_;
  int32_t num_obs_min_;
  double radius_obs_max_;
  double radius_obs_min_;
  int32_t num_obs_max_extra_;
  int32_t num_obs_min_extra_;
  double radius_obs_max_extra_;
  double radius_obs_min_extra_;
  double vibration_distance_extra_;
  vector<geometry_msgs::Point> pos_obs_;
  vector<double> radius_obs_;
  vector<geometry_msgs::Point> pos_obs_extra_;
  vector<double> radius_obs_extra_;
  vector<geometry_msgs::Point> local_goals_;
  vector<geometry_msgs::Point> start_goal_;

  // path
  nav_msgs::Path global_path_;
  nav_msgs::Path global_path_interpolate_;

  // tracking
  double lead_distance_factor_;
  double max_leading_distance_;
  double min_leading_distance_;
  int32_t num_tracking_point_;
  int32_t cur_tracking_idx_;

  // reward
  double distance_reward_allocation_factor_;

  // Subscriber
  ros::Subscriber rviz_goal_sub_;

  // Publisher & tf
  ros::Publisher visual_obs_publisher_;
  ros::Publisher visual_obs_extra_publisher_;
  ros::Publisher visual_local_goals_publisher_;
  ros::Publisher visual_start_goal_publisher_;
  ros::Publisher laser_scan_publisher_;
  ros::Publisher visual_path_publisher_;
  ros::Publisher visual_waypoints_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher reward_publisher_;
  tf2_ros::TransformBroadcaster broadcaster_;
  

  // Servicer
  ros::ServiceServer add_obs_server_;
  ros::ServiceServer reset_map_server_;
  ros::ServiceServer set_map_param_server_;
  ros::ServiceServer set_sensor_param_server_;
  ros::ServiceServer step_server_;

  // Timer
  ros::Timer mainloop_timer_;

  //
  void MainLoopCB(const ros::TimerEvent &event);
  bool ResetMapCB(uav_simulator::ResetMap::Request &req,
                uav_simulator::ResetMap::Response &resp);
  bool StepCB(uav_simulator::Step::Request &req,
            uav_simulator::Step::Response &resp);
  bool AddObstacleCB(uav_simulator::AddObstacle::Request &req,
                   uav_simulator::AddObstacle::Response &resp);
  void RvizGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg_p);

  // reset &update
  bool ResetUavPose();
  bool ResetObstacles();
  bool ResetObstaclesExtra();
  bool UpdateGlobalPath();
  bool UpdateDistanceAngleInfo();
  bool UpdateLaserScan();
  bool UpdateLocalGoals();
  bool DisplayStartGoal();

  // tool
  std::vector<double> GetStateVector(const uav_simulator::State &state_msg);
  uav_simulator::Reward GetReward(const uav_simulator::State &cur_state,
                                  const uav_simulator::State &next_state,
                                  const vector<geometry_msgs::Point> &local_goals, 
                                  const double step_time, const bool is_arrival,
                                  const bool is_crash,
                                  const int32_t step_count);

  // model
  void UpdateModel(uav_simulator::State &state,
                   const uav_simulator::Control control, const double duration);
  void Intergrator(uav_simulator::State &state,
                   const uav_simulator::Control control, const double duration);
  bool IsCrash(const uav_simulator::State &state);
  bool IsArrival(const uav_simulator::State &state);
  bool IsOutRange(const uav_simulator::State &state);
};

#endif /* SIMULATOR_H */
