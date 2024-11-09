#include <point_cloud_object_detection_and_recognition/persistent_cloud_filter.hpp>

namespace pcodar
{
PersistentCloudFilter::PersistentCloudFilter() : outlier_filter_(false)
{
}

void PersistentCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
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

void PersistentCloudFilter::get_config(Config& config)
{
  config.persistant_cloud_filter_radius = outlier_filter_.getRadiusSearch();
  config.persistant_cloud_filter_min_neighbors = outlier_filter_.getMinNeighborsInRadius();
}

}  // namespace pcodar
