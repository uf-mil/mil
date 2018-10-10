#pragma once

#include <mil_msgs/PerceptionObject.h>
#include <unordered_map>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <point_cloud_object_detection_and_recognition/PCODARConfig.h>

namespace pcodar
{
using Config = point_cloud_object_detection_and_recognition::PCODARConfig;

enum class classification
{
  SCAN_THE_CODE,
  SHOOTER,
  TOWER_BUOY,
  BUOY,
  DOCK,
  LARGE_BUOY,
  UNKNOWN,
  SPURIOUS
};

const std::vector<std::string> classification_strings{ "scan_the_code", "shooter",    "tower_buoy", "buoy",
                                                       "dock",          "large_buoy", "unknown",    "spurious" };
using point_t = pcl::PointXYZ;
using point_cloud = pcl::PointCloud<point_t>;
using point_cloud_ptr = point_cloud::Ptr;
using point_cloud_const_ptr = point_cloud::ConstPtr;

using id_label_map = std::unordered_map<uint, std::pair<std::string, std::string>>;
using id_label_map_ptr = std::shared_ptr<id_label_map>;

using cluster_t = pcl::PointIndices;
using clusters_t = std::vector<cluster_t>;

}  // namespace pcodar
