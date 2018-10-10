#pragma once

#include "pcodar_types.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/point_types.h>

namespace pcodar
{
class InputCloudFilter
{
public:
  InputCloudFilter();
  void filter(point_cloud_const_ptr in, point_cloud& pc);
  void set_bounds(point_cloud_ptr bounds);
  void set_robot_footprint(point_cloud const& robot);
  void set_robot_pose(Eigen::Affine3d const& transform);

private:
  point_cloud robot_footprint_;
  pcl::CropHull<pcl::PointXYZ> bounds_filter_;
  pcl::CropHull<pcl::PointXYZ> robot_filter_;
};

}  // namespace pcodar
