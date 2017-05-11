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

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <list>

class Classification
{
  ros::NodeHandle *nh_;

  cv::Point2d get_first_hit(cv::Mat &mat_ogrid, cv::Point2d start, float theta, int max_dis);

public:
  Classification(ros::NodeHandle *nh);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);

  void color_ogrid(cv::Mat &ogrid, tf::StampedTransform &transform);

  void zonify(cv::Mat &mat_ogrid, float resolution, const tf::StampedTransform &transform);
};