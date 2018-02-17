#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObject.h>

#include <limits>

namespace pcodar
{

extern uint NO_ASSOCIATION_FOUND;

struct association_unit
{
    uint index;
    uint object_id;
};

class associator
{
   public:
    associator()
    {
    }
    std::vector<association_unit> associate(const id_object_map& object_map,
                                            const std::vector<mil_msgs::PerceptionObject>& objects);
};

}  // namespace pcod#pragma once
