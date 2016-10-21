#pragma once

#include <navigator_vision_lib/pcd_sub_pub_algorithm.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <ros_camera_stream.hpp>
#include <string>
#include <iostream>

namespace nav{

class PcdColorizer : public PcdSubPubAlgorithm{
  /*
    This class takes adds color information to XYZ only point clouds if the
    points in the cloud can be observed by a camera that takes color images
    Note: needs accurate TF because it uses the ros tf package to transform 
    arbitrary point cloud frames into the frame of the color pinhole camera
    Note: the rgb camera topic should be streaming rectified images
  */

  typedef sensor_msgs::PointCloud2 PCD;

public:
  PcdColorizer(ros::NodeHandle nh, std::string input_pcd_topic, std::string output_pcd_topic, std::string rgb_cam_topic, std::string rgb_cam_frame);
  ~PcdColorizer(){}
  void _transform_to_cam();
  void _color_pcd();  

private:
  std::vector<nav::ROSCameraStream<cv::Vec3b>> ros_cameras;

  void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg);
  std::string rgb_cam_frame;
  std::string rgb_cam_topic;
  tf::TransformListener tf_listener;
  PCD transformed_pcd; // input pcd transformed to the frame of the rgb camera
  image_transport::ImageTransport img_transport{this->_nh};
  image_transport::CameraSubscriber rgb_cam_sub;

 int seq = 0;

  Eigen::Matrix3f cam_intrinsics;
  bool _intrinsics_set = false;
  sensor_msgs::ImageConstPtr latest_frame_img_msg;
  sensor_msgs::CameraInfoConstPtr latest_frame_info_msg;

}; // end class PcdColorizer

} // namespace nav
