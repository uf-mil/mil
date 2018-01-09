#pragma once

#include "pcodar_params.hpp"

#include "point_cloud_builder.hpp"

#include <mil_msgs/PerceptionObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

namespace pcodar
{
class object_detector
{
   public:
    object_detector(const pcodar_params& params) : params_(params), pc_builder_(params){};
    mil_msgs::PerceptionObjectArray get_objects();
    void add_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_enu);


   private:
    pcodar_params params_;
    point_cloud_builder pc_builder_;
};
}
