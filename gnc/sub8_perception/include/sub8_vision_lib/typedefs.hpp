#pragma once
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace sub {
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointXYZRGB PointXYZT;
typedef pcl::PointCloud<PointXYZT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointXYZT> ColorHandlerT;
}