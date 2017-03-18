#pragma once

/*
  This file includes all of the common dependencies for the files in the colorizer directory.
  It also forward declares many of the most commonly used types and type aliases.
*/

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <future>
#include <list>

namespace nav{

template<typename T = pcl::PointXYZ> using PCD = pcl::PointCloud<T>;
template<typename T = pcl::PointXYZ> using PCDPtr = std::shared_ptr<PCD<T>>;
template<typename T> using SPtrVector = std::vector<std::shared_ptr<T>>;
template<typename T> using UPtrVector = std::vector<std::unique_ptr<T>>;

} // namespace nav