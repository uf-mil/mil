#pragma once

#include <mil_msgs/PerceptionObject.h>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcodar
{
enum class classification
{
    SCAN_THE_CODE,
    SHOOTER,
    TOWER_BUOY,
    BUOY,
    DOCK,
    LARGE_BUOY,
    UNKNOWN,
    SPURIOUS
};

const std::vector<std::string> classification_strings{"scan_the_code", "shooter",    "tower_buoy", "buoy",
                                                      "dock",          "large_buoy", "unknown",    "spurious"};
using point_cloud = pcl::PointCloud<pcl::PointXYZ>;
using point_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using id_object_map = std::unordered_map<uint, mil_msgs::PerceptionObject>;
using id_object_map_ptr = std::shared_ptr<id_object_map>;

}  // namespace pcodar