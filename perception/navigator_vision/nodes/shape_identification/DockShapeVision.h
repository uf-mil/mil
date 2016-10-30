#pragma once
#include <navigator_msgs/DockShapes.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
class DockShapeVision {
 protected:
  ros::NodeHandle& nh;
  DockShapeVision(ros::NodeHandle& nh);

 public:
  virtual void GetShapes(cv::Mat& frame, cv::Rect roi,
                         navigator_msgs::DockShapes& symbols) = 0;
  virtual void init() = 0;
};
