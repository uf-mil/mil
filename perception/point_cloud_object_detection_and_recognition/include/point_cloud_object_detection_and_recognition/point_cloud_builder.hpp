#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>

#include <boost/circular_buffer.hpp>

#include <pcl/filters/extract_indices.h>

namespace pcodar
{
point_cloud transform_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_X);
class point_cloud_builder
{
   public:
    point_cloud_builder(bool real_time = true)
        : real_time_(real_time), prev_clouds_(params.number_persistant_point_clouds)
    {
    }
    point_cloud get_point_cloud();
    void add_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_enu);

   private:
    bool real_time_;

    point_cloud mega_cloud_;
    boost::circular_buffer<point_cloud_ptr> prev_clouds_;
};
}  // pcodar namespace
