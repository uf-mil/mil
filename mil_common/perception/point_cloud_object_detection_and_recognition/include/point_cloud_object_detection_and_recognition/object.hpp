#pragma once

#include "pcodar_types.hpp"

namespace pcodar
{
/**
 * Represents an object, a persistent clustered set of points that are nearby. Objects can have associated
 * labels and other metadata.
 */
class Object
{
public:
  /// Create an object from a pointcloud of points associated with
  Object(point_cloud const& pc);
  /// Update the points associated with an object
  void update_points(point_cloud const& pc);
  /// ROS message representing the object, automaticly kept up to date
  /// TODO: make private
  mil_msgs::PerceptionObject msg_;
  /// The points associated with this object
  point_cloud points_;
  /// The center of the minimum area bounding box aroudn the objet
  point_t center_;

private:
  /// Update points_ and center_, called after a call to update_points
  void update_msg();
};

}  // namespace pcodar
