#pragma once

#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>

// TODO:
// De-projected cross-section (real world cross-section)

namespace sub {
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointXYZRGB PointXYZT;
typedef pcl::PointCloud<PointXYZT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointXYZT> ColorHandlerT;

// Determine nearest point to a ray specified by its direction, and the ray's origin
// TODO: Waste computation checking if something is along the RAY and not just the line?
template <typename PointT>
size_t closest_point_index_ray(const pcl::PointCloud<PointT>& cloud,
                               const Eigen::Vector3f& direction, const Eigen::Vector3f& line_pt) {
  // One number greater than all others...one number to rule them all...
  // double min_distance = std::numeric_limits<double>::infinity();
  double min_distance = 10;
  size_t min_index = 0;
  PointT point;
  for (size_t index = 0; index < cloud.points.size(); index++) {
    point = cloud.points[index];

    Eigen::Vector3f cloud_pt(point.x, point.y, point.z);
    // squared norm will be a little bit faster
    double distance =
        ((line_pt - cloud_pt) - ((line_pt - cloud_pt).dot(direction) * direction)).norm();

    if (distance < min_distance) {
      min_distance = distance;
      min_index = index;
    }
  }
  return (min_index);
}

template <typename PointT>
PointT closest_point_ray(const pcl::PointCloud<PointT>& cloud, const Eigen::Vector3f& direction,
                         const Eigen::Vector3f& line_pt) {
  size_t index = closest_point_index_ray(cloud, direction, line_pt);
  return (cloud.points[index]);
}

template <typename PointT>
PointT project_uv_to_cloud(const pcl::PointCloud<PointT>& cloud, const cv::Point2d& image_point,
                           const image_geometry::PinholeCameraModel camera_model) {
  // Assumes camera is at origin, pointed in the normal direction
  PointT pt_pcl;
  cv::Point3d pt_cv;
  pt_cv = camera_model.projectPixelTo3dRay(image_point);
  Eigen::Vector3f direction_eig(pt_cv.x, pt_cv.y, pt_cv.z);
  Eigen::Vector3f origin_eig(0.0, 0.0, 0.0);

  pt_pcl = sub::closest_point_ray(cloud, direction_eig, origin_eig);
  return (pt_pcl);
}
}