#pragma once

#include <mil_msgs/PerceptionObject.h>
#include <unordered_map>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <point_cloud_object_detection_and_recognition/PCODARConfig.h>

namespace pcodar
{
/// Alias for the dyanmic reconfigure object used for PCODAR configuration
using Config = point_cloud_object_detection_and_recognition::PCODARConfig;

/// Alias for type of pointcloud used internally
using point_t = pcl::PointXYZ;
/// Pointcloud of point_t
using point_cloud = pcl::PointCloud<point_t>;
/// Pointer to PCODAR's pointclouds
using point_cloud_ptr = point_cloud::Ptr;
/// Constant poitner to PCODAR's pointclouds
using point_cloud_const_ptr = point_cloud::ConstPtr;

/// Clusters used in object detection
using cluster_t = pcl::PointIndices;
/// Vector of clusters
using clusters_t = std::vector<cluster_t>;

}  // namespace pcodar
