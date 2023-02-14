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
    _mk_msg.scale.x = 16;
    _mk_msg.scale.y = 30;
    
  visualization_msgs::Marker _mk_msg_top;
      _mk_msg_top.header.frame_id = "map";
  _mk_msg_top.header.stamp = ros::Time::now();
  _mk_msg_top.type = visualization_msgs::Marker::CUBE;
  _mk_msg_top.pose.position.x = -16;
  _mk_msg_top.pose.position.y = 0;
  _mk_msg_top.pose.position.z = 0.01;
    _mk_msg_top.pose.orientation.w = 1;
    _mk_msg.id = 1;
  _mk_msg_top.color.r = 1.0;
  _mk_msg_top.color.g = 1.0;
  _mk_msg_top.color.b = 1.0;
  _mk_msg_top.color.a = 1.0;
  _mk_msg_top.action = visualization_msgs::Marker::ADD;
  _mk_msg_top.scale.z = 0.01;
    _mk_msg_top.scale.x = 16;
    _mk_msg_top.scale.y = 31;


  visualization_msgs::Marker _mk_msg_down;
      _mk_msg_down.header.frame_id = "map";
  _mk_msg_down.header.stamp = ros::Time::now();
  _mk_msg_down.type = visualization_msgs::Marker::CUBE;
  _mk_msg_down.pose.position.x = 16;
  _mk_msg_down.pose.position.y = 0;
  _mk_msg_down.pose.position.z = 0.01;
    _mk_msg_down.pose.orientation.w = 1;
    _mk_msg.id = 1;
  _mk_msg_down.color.r = 1.0;
  _mk_msg_down.color.g = 1.0;
  _mk_msg_down.color.b = 1.0;
  _mk_msg_down.color.a = 1.0;
  _mk_msg_down.action = visualization_msgs::Marker::ADD;
  _mk_msg_down.scale.z = 0.01;
    _mk_msg_down.scale.x = 16;
    _mk_msg_down.scale.y = 31;

ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("floor",10);
ros::Publisher pub_top = nh.advertise<visualization_msgs::Marker>("floor_top",10);
ros::Publisher pub_down = nh.advertise<visualization_msgs::Marker>("floor_down",10);

  while (ros::ok()) {
    pub.publish(_mk_msg);
    pub_top.publish(_mk_msg_top);
    pub_down.publish(_mk_msg_down);

    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}