#pragma once

/*
  This file includes all of the common dependencies for the files in the colorizer directory.
  It also forward declares many of the most commonly used types and type aliases.
*/

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <future>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

namespace mil_vision
{
template <typename T = pcl::PointXYZ>
using PCD = pcl::PointCloud<T>;
template <typename T = pcl::PointXYZ>
using PCDPtr = std::shared_ptr<PCD<T>>;
template <typename T>
using SPtrVector = std::vector<std::shared_ptr<T>>;
template <typename T>
using UPtrVector = std::vector<std::unique_ptr<T>>;

}  // namespace mil_vision