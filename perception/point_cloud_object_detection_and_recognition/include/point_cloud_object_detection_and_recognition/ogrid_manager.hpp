#pragma once

#include "pcodar_types.hpp"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace pcodar
{
  class ogrid_manager
  {
    public:
      void initialize(ros::NodeHandle& nh);
      void update_ogrid(const mil_msgs::PerceptionObjectArray &objects, nav_msgs::OdometryConstPtr odom);
    private:
      ros::Publisher pub_ogrid_;
      cv::Mat ogrid_mat_;
      nav_msgs::OccupancyGrid ogrid_;
  };
}
