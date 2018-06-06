#pragma once
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

#include <waypoint_validity.hpp>  // C3

class Classification
{
  ros::NodeHandle *nh_;

  // Usage: Given starting point, and an angle, find the first occupied point in that direction.
  cv::Point2d get_first_hit(cv::Mat &mat_ogrid, cv::Point2d start, float theta, int max_dis);

  float certainty_as_hit_;
  int hit_buffer_;
  float uncertainty_as_hit_;
  float not_hit_degrade_;

public:
  Classification(ros::NodeHandle *nh);

  // Usage: statistical filterting to get rid of outliers and noise
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);

  // Usage: obtain a clustering from pointcloud
  std::vector<pcl::PointIndices> clustering(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);

  /* Usage: Find all the first occupied points in an expanding circle, then color the ogrid
     param mat_ogrid: what ogrid will be used for processing and drawn on
     param resolution: used to convert meters to pixels
     param transform: get subs pose
     param mat_origin: where the center of the ogrid is in resepct to map frame
  */
  void zonify(cv::Mat &mat_ogrid, float resolution, const tf::StampedTransform &transform, cv::Point &mat_origin);
};
