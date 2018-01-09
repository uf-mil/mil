#include <point_cloud_object_detection_and_recognition/object_associator.hpp>

#include <mil_msgs/PerceptionObject.h>

namespace pcodar
{
uint NO_ASSOCIATION_FOUND = std::numeric_limits<uint>::max();

std::vector<association_unit> associator::associate(const id_object_map& object_map,
                                                    const std::vector<mil_msgs::PerceptionObject>& objects)
{
    std::vector<association_unit> association_units;

    for (uint i = 0; i != objects.size(); ++i)
    {
        const auto& object = objects[i];
        double min_distance = std::numeric_limits<double>::max();
        int min_id = -1;
        std::set<uint> associated_ids;

        for (const auto& id_object : object_map)
        {
            if (associated_ids.find(id_object.first) != associated_ids.end())
            {
                continue;
            }

            const double diff_x = object.pose.position.x - id_object.second.pose.position.x;
            const double diff_y = object.pose.position.y - id_object.second.pose.position.y;
            const double diff_z = object.pose.position.z - id_object.second.pose.position.z;

            const double norm = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
            if (norm < min_distance && norm < params_.max_distance_for_association)
            {
                min_distance = norm;
                min_id = i;
            }
        }

        if (min_id == -1)
        {
            association_units.push_back({.index = i, .object_id = NO_ASSOCIATION_FOUND});
        }
        else
        {
            uint id = static_cast<uint>(min_id);
            association_units.push_back({.index = i, .object_id = id});
            associated_ids.insert(id);
        }
    }

    return association_units;
}
}  // namespace pcodar
