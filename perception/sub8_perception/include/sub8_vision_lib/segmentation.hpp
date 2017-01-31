#pragma once
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include "typedefs.hpp"

namespace sub {

// Compute the point in $cloud that is closest to the ray defined by $direction and $line_pt
//
// Compute the minimum over the set of points $cloud: dist(line_pt + direction*t, point)
// TODO:
//  ? Additional (wasteful) branching to check if something is actually in front of the ray (Always
//  true for an image)
//  - Does squaredNorm improve speed? (This will be miniscule at best, but saves computation)
//
// @param[in] target_cloud Cloud of interest
// @param[in] indices Indices of seed points
// @param[out] clusters Indices corresponding to clusters of points
// @param[out] colored_cloud Cloud of colored regions for debugging purposes (Only computed if input
// is not NULL)
template <typename PointT>
void segment_rgb_region_growing(const typename pcl::PointCloud<PointT>::Ptr target_cloud,
                                const std::vector<int>& indices,
                                std::vector<pcl::PointIndices>& clusters,
                                typename pcl::PointCloud<PointT>::Ptr colored_cloud) {
  typename pcl::search::Search<PointT>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
  // typename pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  // Construct a stupid indicesptr
  boost::shared_ptr<std::vector<int> > IndicesPtr(new std::vector<int>(indices));

  // pcl::PassThrough<PointT> pass;
  // pass.setInputCloud(target_cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.0, 1.0);
  // pass.filter(*indices);

  pcl::RegionGrowingRGB<PointT> region_grower;
  region_grower.setInputCloud(target_cloud);
  region_grower.setIndices(IndicesPtr);
  region_grower.setSearchMethod(tree);
  region_grower.setDistanceThreshold(10);
  region_grower.setPointColorThreshold(25);
  region_grower.setRegionColorThreshold(25);
  region_grower.setMinClusterSize(5);

  region_grower.extract(clusters);

  // std::cout << "Cluster Sizes: " << clusters.size() << " " << IndicesPtr->size() << std::endl;
  if (clusters.size() == 0) {
    return;
  }
  colored_cloud = region_grower.getColoredCloud();
  // std::cout << "Constructing colored cloud" << colored_cloud->points.size() << std::endl;
  // std::cout << indices.size() << std::endl;
}

template <typename PointT>
void segment_box(const typename pcl::PointCloud<PointT>::Ptr input_cloud,
                 const Eigen::Vector3f& center, const double edge_length,
                 pcl::PointCloud<PointT>& output_cloud) {
  pcl::CropBox<PointT> box_cropper;
  box_cropper.setInputCloud(input_cloud);

  Eigen::Vector4f h_min_pt, h_max_pt;
  h_min_pt << center.array() - (edge_length / 2.0), 1.0;
  h_max_pt << center.array() + (edge_length / 2.0), 1.0;

  box_cropper.setMin(h_min_pt);
  box_cropper.setMax(h_max_pt);
  box_cropper.filter(output_cloud);
}
}