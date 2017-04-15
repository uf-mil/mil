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


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class Classification
{
  ros::NodeHandle *nh_;

public:
  Classification(ros::NodeHandle *nh);
  void findNormals(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);
  void segment(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);

  void filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud);
  ros::Publisher pubTest;
  ros::Publisher pubTest2;
};