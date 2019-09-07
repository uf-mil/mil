#pragma once

#include "object.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/ObjectDBQuery.h>
#include <mil_msgs/PerceptionObjectArray.h>

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
  bool DatabaseQuery(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response &res);
  /// Internal map of id's to objects
  /// TODO: make private and provide function interfaces
  std::unordered_map<uint, Object> objects_;
  /// Add a new object by its pointcloud, given it a new unique id
  void add_object(point_cloud pc);
  /// The id that will be assigned to the next new object, starting at 0
  size_t highest_id_;
};

}  // namespace pcodar
