#pragma once

#include "object_map.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace pcodar
{
class ogrid_manager
{
public:
  ogrid_manager();
  void initialize(ros::NodeHandle& nh);
  void update_ogrid(ObjectMap const& objects);
  void draw_boundary();
  void update_config(Config const& config);
  void set_bounds(point_cloud_ptr pc);
private:
  cv::Point point_in_ogrid(point_t point);
  double resolution_meters_per_cell_;
  uint32_t width_meters_;
  uint32_t height_meters_;
  uint32_t inflation_cells_;
  ros::Publisher pub_ogrid_;
  cv::Mat ogrid_mat_;
  nav_msgs::OccupancyGrid ogrid_;
  point_cloud_ptr bounds_;
};

}  // namespace pcodar
