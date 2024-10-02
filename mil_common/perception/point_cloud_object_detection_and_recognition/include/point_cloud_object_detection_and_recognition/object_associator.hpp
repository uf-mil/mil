#pragma once

#include <mil_msgs/PerceptionObject.h>
#include <ros/ros.h>

#include <limits>

#include "object_map.hpp"
#include "pcodar_types.hpp"

namespace pcodar
{
/**
 * Associates recently identified objects with previous objects so they persist over time.
 * This is accomplished by finding the nearest neighbor point, creating a new object if this is
 * greater than a maximum distance.
 */
class Associator
{
public:
  /// Update the dynamic reconfigure parameters associated with this class
  void update_config(Config const& config);
  /// Associate old objects with newly identified clusters. @prev_objects is updated + appended in place for new
  /// associations
  void associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters, ros::Time moving_back_at);

private:
  double max_distance_;
  bool forget_unseen_;
};

}  // namespace pcodar
