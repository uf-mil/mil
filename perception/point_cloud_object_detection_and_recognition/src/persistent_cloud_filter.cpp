#include <point_cloud_object_detection_and_recognition/persistent_cloud_filter.hpp>

namespace pcodar
{

PersistentCloudFilter::PersistentCloudFilter():
  outlier_filter_(false)
{
}

void PersistentCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
  outlier_filter_.setInputCloud(in);
  outlier_filter_.filter(pc);
}

void PersistentCloudFilter::update_config(Config const& config)
{
  outlier_filter_.setStddevMulThresh(config.persistant_cloud_filter_stddev);
  outlier_filter_.setMeanK(config.persistant_cloud_filter_mean_k);
}

} // namespace pcodar
