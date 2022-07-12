#pragma once

#include <mil_msgs/ObjectDBQuery.h>
#include <mil_msgs/PerceptionObjectArray.h>

#include "object.hpp"
#include "pcodar_types.hpp"

namespace pcodar
{
/**
 * Contains a map of unique integer identifiers to objects (point clusters).
 */
class ObjectMap
{
public:
  ObjectMap();
  /// Creates a ROS message of the objects to publish / use for markers
  mil_msgs::PerceptionObjectArray to_msg();
  /// Processes a database query service request
  bool DatabaseQuery(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res);
  /// Internal map of id's to objects
  /// TODO: make private and provide function interfaces
  std::unordered_map<uint, Object> objects_;
  using Iterator = decltype(objects_.begin());
  /// Recently removed objects, for marker manager
  std::vector<uint> just_removed_;
  /// Add a new object by its pointcloud, given it a new unique id
  uint add_object(point_cloud_ptr const& pc, KdTreePtr const& search_tree);
  /// Erase an object
  Iterator erase_object(Iterator const& it);
  /// The id that will be assigned to the next new object, starting at 0
  size_t highest_id_;
};

}  // namespace pcodar
