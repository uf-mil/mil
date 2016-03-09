#pragma once

#include <pcl/features/normal_3d_omp.h>
// #define BACKWARD_HAS_BFD 1
// #include <sub8_build_tools/backward.hpp>
#include "geometry.hpp"
#include "segmentation.hpp"
#include "filtering.hpp"
#include "visualization.hpp"
#include "typedefs.hpp"

// TODO:
// De-projected cross-section (real world cross-section)

namespace sub {

template <typename PointT>
void compute_normals(const typename pcl::PointCloud<PointT>::Ptr input_cloud,
                     PointCloudNT& output_cloud, double normal_radius = 0.05) {
  pcl::NormalEstimationOMP<PointNT, PointNT> normal_estimator;
  normal_estimator.setRadiusSearch(normal_radius);
  normal_estimator.setInputCloud(input_cloud);
  normal_estimator.compute(output_cloud);
}
}