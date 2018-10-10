#pragma once

#include "pcodar_types.hpp"

namespace pcodar
{
class Object
{
public:
  Object(point_cloud const& pc);
  void update_points(point_cloud const& pc);
  mil_msgs::PerceptionObject msg_;
  point_cloud points_;
  point_t center_;

private:
  void update_msg();
};

}  // namespace pcodar
