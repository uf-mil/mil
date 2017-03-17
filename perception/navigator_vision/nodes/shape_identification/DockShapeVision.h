#pragma once
#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/DockShape.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
class DockShapeVision {
 protected:
  ros::NodeHandle& nh;
  DockShapeVision(ros::NodeHandle& nh);

 public:
  virtual void GetShapes(cv::Mat& frame, navigator_msgs::DockShapes& symbols) = 0;
  static void DrawShapes(cv::Mat& frame, navigator_msgs::DockShapes& symbols);
  static int fontFace;
  static double fontScale;
  virtual void init() = 0;
};
