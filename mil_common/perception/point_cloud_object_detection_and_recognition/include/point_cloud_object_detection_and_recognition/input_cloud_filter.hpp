#pragma once

#include "pcodar_types.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/point_types.h>

namespace pcodar
{
/**
 * Filters a single incoming pointcloud after it has been transformed into global frame.
 * Removes points outside of a specified "bounds" polygon and points inside a specified robot footprint polygon.
 */
class InputCloudFilter
{
public:
  InputCloudFilter();
  /// Filters @in, storing resulting pointcloud into @pc
  void filter(point_cloud_const_ptr in, point_cloud& pc);
  /// Sets the bounds with a pointcloud where each point is a verticie of the bounds polygon
  void set_bounds(point_cloud_ptr bounds);
  /// Set the robot footprint with a pointcloud in the robot's local frame where each point is a verticie of the
  /// footprint polygon
  void set_robot_footprint(point_cloud const& robot);
  /// Sets the transform from the global frame to the robot, should be called before @filter
  void set_robot_pose(Eigen::Affine3d const& transform);

private:
  point_cloud robot_footprint_;
  pcl::CropHull<pcl::PointXYZ> bounds_filter_;
  pcl::CropHull<pcl::PointXYZ> robot_filter_;
};

}  // namespace pcodar
