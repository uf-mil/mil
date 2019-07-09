#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

void FitPlanesToCloud(objectMessage &object, sensor_msgs::PointCloud &rosCloud, const geometry_msgs::Pose &boatPose_enu)
{
  // Check that we have enough points to run model
  // if (object.scale.x < 4 || object.scale.y < 4 || object.scale.z < 2 || object.strikesFrame.size() <= 100) {
  if (object.name != "shooter" && object.name != "dock")
  {
    return;
  }
  ROS_INFO_STREAM("PLANE FIT | Running plane fit on object "
                  << object.id << ", height " << object.scale.z << ", inliers " << object.pclInliers << ", normal "
                  << object.normal.x << "," << object.normal.y << "," << object.normal.z);

  // Create pcl data types
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>),
      pclCloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
  pclCloud->height = 1;  // 1D data set instead of 2D

  // Downsample while adding to pcl pointcloud
  double voxel = 0.15;
  for (const auto &nextPoint : object.strikesFrame)
  {
    if (nextPoint.z > object.position.z - object.scale.z * 0.05)
    {
      pclCloud->push_back(pcl::PointXYZ(nextPoint.x, nextPoint.y, nextPoint.z));
    }
  }

  // Update cloud width
  pclCloud->width = pclCloud->points.size();

  // Setup plane model
  pcl::ModelCoefficients coeffs;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> segmenter;
  // segmenter.setOptimizeCoefficients(true);
  segmenter.setEpsAngle(0.75f * M_PI / 180.f);
  segmenter.setModelType(
      pcl::SACMODEL_PARALLEL_PLANE);  // Documentation is confusing/wrong here... looking for parallel works...
  segmenter.setAxis(Eigen::Vector3f(0, 0, 1));
  segmenter.setMethodType(pcl::SAC_RANSAC);
  segmenter.setDistanceThreshold(0.15);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Segment the largest planar component from the remaining cloud
  segmenter.setInputCloud(pclCloud);
  try
  {
    segmenter.segment(*inliers, coeffs);
  }
  catch (...)
  {
  }
  if (inliers->indices.size() < 4)
  {
    ROS_INFO_STREAM("PLANE FIT | A suitable plane wasn't found...");
    // break;
  }
  else if (inliers->indices.size() < object.pclInliers * 0.75)
  {
    ROS_INFO_STREAM("PLANE FIT | A plane was found but has less points than previous...");
  }
  else
  {
    ROS_INFO_STREAM("PLANE FIT | Changing pclInliears to " << inliers->indices.size() << " from " << object.pclInliers);
    object.pclInliers = inliers->indices.size();
    for (auto ii : inliers->indices)
    {
      geometry_msgs::Point32 p32;
      p32.x = pclCloud->points[ii].x;
      p32.y = pclCloud->points[ii].y;
      p32.z = pclCloud->points[ii].z;
      rosCloud.points.push_back(p32);
      rosCloud.channels[0].values.push_back(100);
    }
    ROS_INFO_STREAM("PLANE FIT | " << inliers->indices.size() << " out of " << pclCloud->points.size() << " : "
                                   << rosCloud.points.size());
    ROS_INFO_STREAM("PLANE FIT | Coeffs found: " << coeffs.values[0] << "," << coeffs.values[1] << ","
                                                 << coeffs.values[2] << " - " << inliers->indices.size());
    // Set object normal to best normal found
    object.normal.x = coeffs.values[0];
    object.normal.y = coeffs.values[1];
    object.normal.z = coeffs.values[2];
  }

  if (object.normal.x != 0 && object.normal.y != 0 && object.normal.z != 0)
  {
    // Update the current object normal to face the boat
    Eigen::Vector3d bestNormal, normal, v_object_boat;
    normal(0) = object.normal.x;
    normal(1) = object.normal.y;
    normal(2) = object.normal.z;
    normal.normalize();
    bestNormal = normal;
    v_object_boat(0) = boatPose_enu.position.x - object.position.x;
    v_object_boat(1) = boatPose_enu.position.y - object.position.y;
    v_object_boat(2) = boatPose_enu.position.z - object.position.z;
    v_object_boat.normalize();

    // std::cout << v_object_boat(0) << "," << v_object_boat(1) << "," << v_object_boat(2) << std::endl;
    // std::cout << normal(0) << "," << normal(1)  << "," << normal(2) << std::endl;

    double angle = fabs(acos(normal.dot(v_object_boat)) * 180.0 / M_PI);
    // Rotate the normal 3 times to get the other directions at 90 degree seperation
    for (int ii = 1; ii <= 3; ++ii)
    {
      Eigen::Affine3d RotateZ(Eigen::Affine3d::Identity());
      RotateZ.rotate(Eigen::AngleAxisd(M_PI / 2.0 * ii, Eigen::Vector3d::UnitZ()));
      Eigen::Vector3d rotNormal = RotateZ * normal;
      // std::cout << rotNormal(0) << "," << rotNormal(1) << "," << rotNormal(2) << std::endl;
      double rotAngle = fabs(acos(rotNormal.dot(v_object_boat)) * 180.0 / M_PI);
      if (rotAngle < angle)
      {
        angle = rotAngle;
        bestNormal = rotNormal;
      }
    }

    // Set object normal to best normal found
    object.normal.x = bestNormal(0);
    object.normal.y = bestNormal(1);
    object.normal.z = bestNormal(2);
  }
}

// Extract the inliers
/*extract.setInputCloud(pclCloud);
extract.setIndices(inliers);
extract.setNegative(true);
extract.filter(*pclCloud_seg);
pclCloud.swap(pclCloud_seg);*/