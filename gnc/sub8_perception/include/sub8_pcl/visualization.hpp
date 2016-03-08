#pragma once

#include "geometry.hpp"
#include "segmentation.hpp"
#include "typedefs.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

namespace sub {

// ******* 3D Visualization *******
class RvizVisualizer {
 public:
  ros::Publisher buoy_pub;
  ros::NodeHandle nh;
  RvizVisualizer();
  void visualize_buoy(geometry_msgs::Pose &pose, std::string &frame_id);
};
}