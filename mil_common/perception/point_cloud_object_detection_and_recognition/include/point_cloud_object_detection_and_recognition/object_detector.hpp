#pragma once

#include <mil_msgs/PerceptionObjectArray.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcodar_types.hpp"

namespace pcodar
{
/**
 * Detects objects in a pointcloud by clustering points that are nearby.
 */
class ObjectDetector
{
public:
  /// Returns an array of clusters found in @pc
  clusters_t get_clusters(point_cloud_const_ptr pc);
  /// Update the dynamic reconfigure parameters associated with this class
  void update_config(Config const& config);
  void get_config(Config& config);

private:
  pcl::EuclideanClusterExtraction<point_t> cluster_extractor_;
};
}  // namespace pcodar
