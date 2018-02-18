#pragma once

#include "pcodar_types.hpp"
#include "pcodar_params.hpp"

#include <mil_msgs/PerceptionObject.h>

#include <sensor_msgs/PointCloud2.h>

namespace pcodar
{
std::vector<mil_msgs::PerceptionObject> get_point_cloud_clusters(const point_cloud& pcloud);
}  // namespace pcodar
