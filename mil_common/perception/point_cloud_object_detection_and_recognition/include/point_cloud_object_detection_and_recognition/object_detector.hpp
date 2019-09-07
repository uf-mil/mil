#pragma once

#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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

private:
  pcl::EuclideanClusterExtraction<point_t> cluster_extractor_;
};
}
