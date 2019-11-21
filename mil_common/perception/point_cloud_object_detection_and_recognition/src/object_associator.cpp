#include <point_cloud_object_detection_and_recognition/object_associator.hpp>

#include <mil_msgs/PerceptionObject.h>
#include <pcl/search/octree.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcodar
{
void Associator::associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters)
{
  // Iterate through each new cluster, finding which persistent cluster(s) it matches
  for (cluster_t const& cluster : clusters)
  {
    // Make pointcloud from this pointcloud
    point_cloud_ptr cluster_pc = boost::make_shared<point_cloud>(pc, cluster.indices);
    KdTreePtr cluster_search_tree = boost::make_shared<KdTree>();
    cluster_search_tree->setInputCloud(cluster_pc);

    using CorrespondenceEstimation = pcl::registration::CorrespondenceEstimation<point_t, point_t>;
    CorrespondenceEstimation ce;
    ce.setSearchMethodTarget(cluster_search_tree, true);
    ce.setInputTarget(cluster_pc);

    using ObjectMapIt = decltype(prev_objects.objects_.begin());
    std::vector<ObjectMapIt> matches;
    for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end(); ++pair)
    {
      ce.setSearchMethodSource((*pair).second.get_search_tree(), true);
      ce.setInputSource((*pair).second.get_points_ptr());

      pcl::CorrespondencesPtr correspondences = boost::make_shared<pcl::Correspondences>();
      ce.determineCorrespondences(*correspondences, max_distance_);

      if (correspondences->size() > 0)
        matches.push_back(pair);
    }

    if (matches.size() == 0) {
      // Add to object
      prev_objects.add_object(cluster_pc, cluster_search_tree);
    } else {
      (*matches.at(0)).second.update_points(cluster_pc, cluster_search_tree);
      for(size_t i = 1; i < matches.size(); ++i) {
        prev_objects.erase_object(matches.at(i));
      }
    }
  }
}

void Associator::update_config(Config const& config)
{
  max_distance_ = config.associator_max_distance;
}

}  // namespace pcodar
