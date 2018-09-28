#pragma once

#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcodar
{
class object_detector
{
public:
    object_detector() {};
    clusters_t get_clusters(point_cloud_const_ptr pc);
    void update_config(Config const& config);
private:
    static double nearest_pair_distance(point_cloud const& pc1, point_cloud const& pc2);
    pcl::EuclideanClusterExtraction<point_t> cluster_extractor_;
};
}
