#pragma once

// #include "geometry.hpp"
// #include "segmentation.hpp"
// #include "typedefs.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

namespace sub {

// ******* 3D Visualization *******
class RvizVisualizer {
 public:
  ros::Publisher rviz_pub;
  ros::NodeHandle nh;
  RvizVisualizer(std::string rviz_topic);
  void visualize_buoy(geometry_msgs::Pose &pose, std::string &frame_id);
  void visualize_torpedo_board(geometry_msgs::Pose& pose, Eigen::Quaterniond orientation, 
  							   std::vector<Eigen::Vector3d>& targets, std::vector<Eigen::Vector3d>& corners3d, 
  							   std::string& frame_id);
};
}