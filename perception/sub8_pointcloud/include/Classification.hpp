#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/point_cloud.h"

#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "opencv2/opencv.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <waypoint_validity.hpp> // C3

class Classification
{
  ros::NodeHandle *nh_;

  // Usage: Given starting point, and an angle, find the first occupied point in that direction.
  cv::Point2d get_first_hit(cv::Mat &mat_ogrid, cv::Point2d start, float theta, int max_dis);

public:
  Classification(ros::NodeHandle *nh);

  // Usage: statistical filterting to get rid of outliers and noise
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);

  // Usage: Find all the first occupied points in an expanding circle, then color the ogrid
  void zonify(cv::Mat &mat_ogrid, float resolution, const tf::StampedTransform &transform);

  // Usage: Draw a fake ogrid to test in SIM
  void fake_ogrid(cv::Mat &mat_ogrid, float resolution, const tf::StampedTransform &transform);
};