#pragma once

#include "pcodar_types.hpp"
#include "object.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

namespace pcodar
{

class ObjectMap
{

public:
  ObjectMap();
  mil_msgs::PerceptionObjectArray to_msg();
  std::unordered_map<uint, Object> objects_;
  void add_object(point_cloud pc);
  size_t highest_id_;
};

} // namespace pcodar
