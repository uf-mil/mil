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
    virtual void GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols) = 0;
    virtual void init() = 0;
};
