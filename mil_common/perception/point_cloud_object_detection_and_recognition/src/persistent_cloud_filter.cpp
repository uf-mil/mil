#include <point_cloud_object_detection_and_recognition/persistent_cloud_filter.hpp>

namespace pcodar
{
PersistentCloudFilter::PersistentCloudFilter() : outlier_filter_(false)
{
}

void PersistentCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
  if (!in)
    std::cout << "in not set" << std::endl;
  if (in->empty())
  {
    pc.clear();
    return;
  }
  outlier_filter_.setInputCloud(in);
  outlier_filter_.filter(pc);
}

void PersistentCloudFilter::update_config(Config const& config)
{
  outlier_filter_.setRadiusSearch(config.persistant_cloud_filter_radius);
  outlier_filter_.setMinNeighborsInRadius(config.persistant_cloud_filter_min_neighbors);
}

}  // namespace pcodar
