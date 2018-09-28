#include <point_cloud_object_detection_and_recognition/object_detector.hpp>
#include <tf/transform_datatypes.h>

namespace pcodar
{

clusters_t object_detector::get_clusters(point_cloud_const_ptr pc)
{

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pc);

  clusters_t cluster_indices;

  cluster_extractor_.setSearchMethod(tree);
  cluster_extractor_.setInputCloud(pc);
  cluster_extractor_.extract(cluster_indices);

  return cluster_indices;
}

void object_detector::update_config(Config const& config)
{
  ROS_INFO("Cluster tolerance=%f min=%d max=%d", config.cluster_tolerance_m, config.cluster_min_points, config.cluster_max_points);
  cluster_extractor_.setClusterTolerance(config.cluster_tolerance_m);
  cluster_extractor_.setMinClusterSize(config.cluster_min_points);
  cluster_extractor_.setMaxClusterSize(config.cluster_max_points);
}

} // namespace pcodar
