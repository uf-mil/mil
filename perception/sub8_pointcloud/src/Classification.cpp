// TODO: Segmentation, Classification, Bounds, Ogrid filtering

#include "Classification.hpp"
Classification::Classification(ros::NodeHandle *nh)
{
  nh_ = nh;

  // TODO: Algorithms to filter pointcloud and possibly some classification scheme
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Classification::filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  if (pointCloud->points.size() < 1)
    return cloud_filtered;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(pointCloud);
  sor.setMeanK(75);
  sor.setStddevMulThresh(.75);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}