#pragma once

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>

#include "pcodar_types.hpp"

namespace pcodar
{
class PersistentCloudFilter
{
public:
  PersistentCloudFilter();
  void filter(point_cloud_const_ptr in, point_cloud& pc);
  void update_config(Config const& config);

private:
  pcl::RadiusOutlierRemoval<point_t> outlier_filter_;
};

}  // namespace pcodar
