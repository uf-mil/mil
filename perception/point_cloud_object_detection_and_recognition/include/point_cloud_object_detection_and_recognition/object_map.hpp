#pragma once

#include "object.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/ObjectDBQuery.h>
#include <mil_msgs/PerceptionObjectArray.h>

namespace pcodar
{
class ObjectMap
{
public:
  ObjectMap();
  mil_msgs::PerceptionObjectArray to_msg();
  bool DatabaseQuery(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response &res);
  std::unordered_map<uint, Object> objects_;
  void add_object(point_cloud pc);
  size_t highest_id_;
};

}  // namespace pcodar
