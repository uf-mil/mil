#pragma once
#include <mil_blueview_driver/BlueViewPing.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <opencv2/core/core.hpp>
#include <stdexcept>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/circular_buffer.hpp>

#include <sub8_msgs/Bounds.h>

#include <waypoint_validity.hpp>

#include <Classification.hpp>

class OGridGen
{
public:
  OGridGen();
  void publish_ogrid(const ros::TimerEvent &);

  void callback(const mil_blueview_driver::BlueViewPingPtr &ping_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_to_imaging_sonar_;

  tf::TransformListener listener_;

  ros::Publisher pub_grid_;
  ros::Publisher pub_point_cloud_filtered_;
  ros::Publisher pub_point_cloud_raw_;
  ros::Timer timer_;

  cv::Mat mat_ogrid_;
  float ogrid_size_;
  float resolution_;
  float pool_depth_;
  int min_intensity_;

  ros::ServiceClient service_get_bounds_;
  tf::StampedTransform transform_;

  boost::circular_buffer<pcl::PointXYZI> point_cloud_buffer_;

  std::vector<cv::Point> bounds_;

  Classification classification_;
};