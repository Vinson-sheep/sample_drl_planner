#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

int32_t main(int32_t argc, char *argv[]) {
  srand(time(NULL));
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "floor_pub");
  ros::NodeHandle nh;

    visualization_msgs::Marker _mk_msg;
      _mk_msg.header.frame_id = "map";
  _mk_msg.header.stamp = ros::Time::now();
  _mk_msg.type = visualization_msgs::Marker::CUBE;
  _mk_msg.pose.position.z = -0.01;
    _mk_msg.pose.orientation.w = 1;
    _mk_msg.id = 0;
  _mk_msg.color.r = 0.546875;
  _mk_msg.color.g = 0.78515625;
  _mk_msg.color.b = 1.0;
  _mk_msg.color.a = 1.0;
  _mk_msg.action = visualization_msgs::Marker::ADD;
  _mk_msg.scale.z = 0.01;
    _mk_msg.scale.x = 15;
    _mk_msg.scale.y = 30;
    
ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("floor",10);

  while (ros::ok()) {
    pub.publish(_mk_msg);

    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}