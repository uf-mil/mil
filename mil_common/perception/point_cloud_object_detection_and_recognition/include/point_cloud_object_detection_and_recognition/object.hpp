#pragma once

#include "pcodar_types.hpp"


namespace mil_gazebo {
class PCODARGazebo;
}

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
  Object(point_cloud_ptr const& pc, uint id);
  /// Update the points associated with an object
  void update_points(point_cloud_ptr const& pc);
  point_cloud const& get_points() const;
  point_cloud_ptr get_points_ptr() const;
  mil_msgs::PerceptionObject const& as_msg() const;
  point_t const& get_center() const;
  void set_classification(std::string const& classification);
  void set_id(uint id);
private:
  /// ROS message representing the object, automaticly kept up to date
  /// TODO: make private
  mil_msgs::PerceptionObject msg_;
  /// The points associated with this object
  point_cloud_ptr points_;
  /// The center of the minimum area bounding box aroudn the objet
  point_t center_;
  /// Update points_ and center_, called after a call to update_points
  void update_msg();

  friend class mil_gazebo::PCODARGazebo;
};

}  // namespace pcodar
