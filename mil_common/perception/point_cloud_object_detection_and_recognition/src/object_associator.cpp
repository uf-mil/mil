#include <point_cloud_object_detection_and_recognition/object_associator.hpp>

#include <mil_msgs/PerceptionObject.h>
#include <pcl/search/octree.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcodar
{
void Associator::associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters)
{
/* Not needed?
  if (prev_objects.objects_.empty())
  {
    for (cluster_t const& cluster : clusters)
    {
    point_cloud_ptr cluster_pc = boost::make_shared<point_cloud>(pc, cluster.indices);

      point_cloud cluster_pc(pc, cluster.indices);
      prev_objects.add_object(cluster_pc);
    }
  }
*/

  /* TODO
   * TREE-IFY each
   * pcl::registration::CorrespondenceEstimation to find point closeness
   * pcl::registration::CorrespondenceRejectorTrimmed to just look for cloest points
   * if one new cluster matches to multiple old clusters -> merge old clusters
   * if one old cluster matches to multiple new clusters -> split old clusters
   */

  pcl::search::Octree<point_t> search(0.01);
  for (cluster_t const& cluster : clusters)
  {
    point_cloud_ptr cluster_pc = boost::make_shared<point_cloud>(pc, cluster.indices);

    using ObjectMapIt = decltype(prev_objects.objects_.begin());
    std::vector<ObjectMapIt> matches;
    for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end(); ++pair)
    {
      using CorrespondenceEstimation = pcl::registration::CorrespondenceEstimation<point_t, point_t>;
      CorrespondenceEstimation ce;
      //pcl::registration::CorrespondenceRejectorTrimmed rt;

      ce.setInputTarget(cluster_pc);
      ce.setInputSource((*pair).second.get_points_ptr());

      pcl::CorrespondencesPtr correspondences = boost::make_shared<pcl::Correspondences>();
      ce.determineCorrespondences(*correspondences, max_distance_);

      if (correspondences->size() > 0)
        matches.push_back(pair);
    }

    if (matches.size() == 0) {
      // Add to object
      prev_objects.add_object(cluster_pc);
    } else {
      (*matches.at(0)).second.update_points(cluster_pc);
      for(size_t i = 1; i < matches.size(); ++i) {
        prev_objects.objects_.erase(matches.at(i));
      }
    }
  }
}

void Associator::update_config(Config const& config)
{
  max_distance_ = config.associator_max_distance;
}

}  // namespace pcodar
