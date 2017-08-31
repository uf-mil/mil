#pragma once

#include "pcodar_params.hh"
#include "pcodar_types.hh"

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
    associator(const pcodar_params& params) : params_(params)
    {
    }
    std::vector<association_unit> associate(const id_object_map& object_map,
                                            const std::vector<mil_msgs::PerceptionObject>& objects);

   private:
    pcodar_params params_;
};

}  // namespace pcod#pragma once
