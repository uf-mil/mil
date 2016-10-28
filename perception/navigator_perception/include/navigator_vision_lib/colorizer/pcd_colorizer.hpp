#pragma once

#include <navigator_vision_lib/pcd_sub_pub_algorithm.hpp>
#include <navigator_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <navigator_vision_lib/colorizer/color_observation.hpp>
#include <navigator_tools.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>

namespace nav{

class PcdColorizer : public PcdSubPubAlgorithm{
  /*
    This class takes adds color information to XYZ only point clouds if the
    points in the cloud can be observed by a camera that takes color images
    Note: needs accurate TF because it uses the ros tf package to transform 
    arbitrary point cloud frames into the frame of the color pinhole camera
    Note: the rgb camera topic should be streaming rectified images
  */

public:

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using CamStream = nav::ROSCameraStream<cv::Vec3b>;
  using CamStreamPtr = std::shared_ptr<CamStream>;

  PcdColorizer(ros::NodeHandle nh, std::string input_pcd_topic);
  ~PcdColorizer(){}
  void _transform_to_cam();
  void _color_pcd();  

private:
  std::vector<CamStreamPtr> _ros_cam_ptrs;
  std::vector<PointCloud::Ptr> _transformed_cloud_ptrs;

  void cloud_cb(const PointCloud::ConstPtr &cloud_in);
  tf::TransformListener _tf_listener;

  // Parameters
  size_t _img_hist_size = 10;

 int seq = 0;

 bool _initialized = false;

}; // end class PcdColorizer

} // namespace nav
