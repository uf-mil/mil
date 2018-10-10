#pragma once

#include "pcodar_types.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <boost/circular_buffer.hpp>

#include <pcl/filters/extract_indices.h>

namespace pcodar
{
class point_cloud_builder
{
public:
  point_cloud_builder();
  point_cloud_ptr get_point_cloud();
  void add_point_cloud(const point_cloud_ptr& pc);
  void update_config(Config const& config);

private:
  point_cloud_ptr mega_cloud_;
  boost::circular_buffer<point_cloud_ptr> prev_clouds_;
};
}  // pcodar namespace
