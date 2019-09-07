#pragma once

#include "object_map.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObject.h>

#include <limits>

namespace pcodar
{
/**
 * Associates recently identified objects with previous objects so they persist over time.
 * This is acomplished by finding the nearest neighbor point, creating a new object if this is
 * greater than a maximum distance.
 */
class Associator
{
public:
  /// Update the dynamic reconfigure parameters associated with this class
  void update_config(Config const& config);
  /// Associate old objects with newly identified clusters. @prev_objects is updated + appended in place for new
  /// associations
  void associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters);

private:
  double max_distance_;
};

}  // namespace pcodar
