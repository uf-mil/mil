#pragma once
#include "typedefs.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace sub {

template <typename PointT>
void voxel_filter(const typename pcl::PointCloud<PointT>::Ptr input_cloud,
                  const typename pcl::PointCloud<PointT>::Ptr output_cloud, float leaf_size) {
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*output_cloud);
}

template <typename PointT>
void statistical_outlier_filter(const typename pcl::PointCloud<PointT>::Ptr input_cloud,
                                const typename pcl::PointCloud<PointT>::Ptr output_cloud,
                                float mean_k, float std_dev_mul_thresh) {
  pcl::StatisticalOutlierRemoval<PointT> outlier_remover;
  outlier_remover.setInputCloud(input_cloud);
  outlier_remover.setMeanK(mean_k);
  outlier_remover.setStddevMulThresh(std_dev_mul_thresh);
  outlier_remover.filter(*output_cloud);
}
}