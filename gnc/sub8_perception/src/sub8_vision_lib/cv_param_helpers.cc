#include <sub8_vision_lib/cv_tools.hpp>
#include <ros/console.h>

namespace sub {

/// Range is an output
void range_from_param(std::string &param_root, Range &range) {
  /// Ex: param_name = /buoy/red --

  // No overhead in doing this
  ros::NodeHandle nh;
  ROS_INFO("-- Waiting for parameter %s, (No timeout)", param_root.c_str());
  while (!nh.hasParam(param_root + "/hsv_low") && ros::ok()) {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-- Got %s", param_root.c_str());

  std::vector<int> v_lower;
  std::vector<int> v_upper;
  nh.getParam(param_root + "/hsv_low", v_lower);
  nh.getParam(param_root + "/hsv_high", v_upper);

  range.lower = cv::Scalar(v_lower[0], v_lower[1], v_lower[2]);
  range.upper = cv::Scalar(v_upper[0], v_upper[1], v_upper[2]);
}

void inParamRange(cv::Mat &src, Range &range, cv::Mat &dest) {
  cv::inRange(src, range.lower, range.upper, dest);
}
}