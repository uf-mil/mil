#pragma once
#include "typedefs.hpp"
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>
#include <limits>

namespace sub {

// TODO: "further" -- point in xyz, uv for camera_n --> a point $dist further along that ray

// OMP: Compute the point in $cloud that is closest to the ray defined by $direction and $line_pt
// (For a pc with n cores, this has ~1/nth the runtime of the non-OMP version)
// (Uses OpenMP to compute in parallel)
// Compute the minimum over the set of points $cloud: dist(line_pt + direction*t, point)
// TODO:
//  ? Additional (wasteful) branching to check if something is actually in front of the ray (Always
//  true for an image)
//  - Does squaredNorm improve speed? (This will be miniscule at best, but saves computation)
//
// @param[in] cloud The point cloud over which to determine the closest point
// @param[in] direction The direction of the ray with which we are concerned
// @param[in] line_pt A point on the line, ideally the origin of the ray
// @return The index of the point in the target point cloud that is closest to the ray of interest
template <typename PointT>
size_t closest_point_index_rayOMP(const pcl::PointCloud<PointT>& cloud,
                                  const Eigen::Vector3f& direction_pre,
                                  const Eigen::Vector3f& line_pt) {
  Eigen::Vector3f direction = direction_pre / direction_pre.norm();
  double min_distance = std::numeric_limits<double>::infinity();
  size_t min_index = 0;
  PointT point;
  std::vector<double> distances(cloud.points.size(), 10);  // Initialize to value 10
// Generate a vector of distances
#pragma omp parallel for
  for (size_t index = 0; index < cloud.points.size(); index++) {
    point = cloud.points[index];

    Eigen::Vector3f cloud_pt(point.x, point.y, point.z);
    Eigen::Vector3f difference = (line_pt - cloud_pt);
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
    double distance = (difference - (difference.dot(direction) * direction)).norm();
    distances[index] = distance;
  }

  // Find index of maximum (TODO: Figure out how to OMP this)
  for (size_t index = 0; index < cloud.points.size(); index++) {
    const double distance = distances[index];
    if (distance < min_distance) {
      min_distance = distance;
      min_index = index;
    }
  }

  return (min_index);
}

// Compute the point in $cloud that is closest to the ray defined by $direction and $line_pt
//
// Compute the minimum over the set of points $cloud: dist(line_pt + direction*t, point)
// TODO:
//  ? Additional (wasteful) branching to check if something is actually in front of the ray (Always
//  true for an image)
//  - Does squaredNorm improve speed? (This will be miniscule at best, but saves computation)
//
// @param[in] cloud The point cloud over which to determine the closest point
// @param[in] direction The direction of the ray with which we are concerned
// @param[in] line_pt A point on the line, ideally the origin of the ray
// @return The index of the point in the target point cloud that is closest to the ray of interest
template <typename PointT>
size_t closest_point_index_ray(const pcl::PointCloud<PointT>& cloud,
                               const Eigen::Vector3f& direction_pre,
                               const Eigen::Vector3f& line_pt) {
  // One number greater than all others...one number to rule them all...
  Eigen::Vector3f direction = direction_pre / direction_pre.norm();
  double min_distance = std::numeric_limits<double>::infinity();
  size_t min_index = 0;
  for (size_t index = 0; index < cloud.points.size(); index++) {
    const PointT point = cloud.points[index];

    Eigen::Vector3f cloud_pt(point.x, point.y, point.z);
    Eigen::Vector3f difference = (line_pt - cloud_pt);
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
    double distance = (difference - (difference.dot(direction) * direction)).norm();

    if (distance < min_distance) {
      min_distance = distance;
      min_index = index;
    }
  }
  return (min_index);
}

// Determine the nearest point to a ray, and return the actual point
template <typename PointT>
PointT closest_point_ray(const pcl::PointCloud<PointT>& cloud, const Eigen::Vector3f& direction,
                         const Eigen::Vector3f& line_pt) {
  size_t index = closest_point_index_rayOMP(cloud, direction, line_pt);
  return (cloud.points[index]);
}

// Get the index of the point closest to the ray deprojected from a point (u, v) in an image
template <typename PointT>
size_t project_uv_to_cloud_index(const pcl::PointCloud<PointT>& cloud,
                                 const cv::Point2d& image_point,
                                 const image_geometry::PinholeCameraModel camera_model) {
  // Assumes camera is at origin, pointed in the normal direction
  size_t pt_pcl_index;
  cv::Point3d pt_cv;
  pt_cv = camera_model.projectPixelTo3dRay(image_point);
  Eigen::Vector3f direction_eig(pt_cv.x, pt_cv.y, pt_cv.z);
  Eigen::Vector3f origin_eig(0.0, 0.0, 0.0);

  pt_pcl_index = closest_point_index_rayOMP(cloud, direction_eig, origin_eig);
  return (pt_pcl_index);
}

// Given pixel coordinates and an image_geometry camera model, return the pcl point closest to the
// projection of that pixel into 3D
template <typename PointT>
PointT project_uv_to_cloud(const pcl::PointCloud<PointT>& cloud, const cv::Point2d& image_point,
                           const image_geometry::PinholeCameraModel camera_model) {
  PointT pt_pcl;
  size_t pt_index = project_uv_to_cloud_index(cloud, image_point, camera_model);
  pt_pcl = cloud.points[pt_index];
  return (pt_pcl);
}

// Convert a pcl point to an eigen vector3f
template <typename PointT>
Eigen::Vector3f point_to_eigen(const PointT& pcl_point) {
  Eigen::Vector3f eig_point(pcl_point.x, pcl_point.y, pcl_point.z);
  return (eig_point);
}
}