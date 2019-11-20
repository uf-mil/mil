#include <point_cloud_object_detection_and_recognition/object_associator.hpp>

#include <mil_msgs/PerceptionObject.h>
#include <pcl/search/octree.h>

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
    search.setInputCloud(cluster_pc);
    float min = std::numeric_limits<float>::max();
    std::unordered_map<uint, Object>::iterator it = prev_objects.objects_.end();
    for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end(); ++pair)
    {
      int index = 0;
      float distance = 0.;
      search.approxNearestSearch((*pair).second.get_center(), index, distance);
      // Search returns squared distance, so sqrt it here
      distance = sqrt(distance);
      if (distance < min && distance < max_distance_)
      {
        min = distance;
        it = pair;
      }
    }

    if (it == prev_objects.objects_.end())
    {
      // Add to object
      prev_objects.add_object(cluster_pc);
    }
    else
    {
      (*it).second.update_points(cluster_pc);
    }
  }
}

void Associator::update_config(Config const& config)
{
  max_distance_ = config.associator_max_distance;
}

}  // namespace pcodar
