#pragma once
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <navigator_msgs/DockShapes.h>
class DockShapeVision
{
  protected:
    ros::NodeHandle& nh;
    DockShapeVision(ros::NodeHandle& nh);
  public:
    void GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols) {};
};
