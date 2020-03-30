#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mil_tools/mil_tools.hpp>

int main(int argc, char** argv)
{
  // TODO: findout exactly what the controler would like instead of pulbshing
  //  every message on the topic that we happened to record
  ros::init(argc, argv, "path_recorder");
  ros::NodeHandle nh("~");
  mil_tools::TopicPlayer<nav_msgs::Odometry> path_player(&nh);
  ros::spin();
  return 0;
}
