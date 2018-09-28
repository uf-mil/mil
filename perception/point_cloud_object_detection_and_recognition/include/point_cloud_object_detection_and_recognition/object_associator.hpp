#pragma once

#include "pcodar_types.hpp"
#include "object_map.hpp"

#include <mil_msgs/PerceptionObject.h>

#include <limits>

namespace pcodar
{

class associator
{
public:
    associator()
    {
    }
    void update_config(Config const& config);
    void associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters);
private:
    double max_distance_;
};

}  // namespace pcodar
