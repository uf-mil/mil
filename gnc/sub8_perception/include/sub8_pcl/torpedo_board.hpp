#pragma once
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>
#include <sub8_pcl/torpedo_board.hpp>
#include "sub8_msgs/VisionRequest.h"

class Sub8TorpedoBoardDetector {
public:
  Sub8TorpedoBoardDetector();
  ~Sub8TorpedoBoardDetector();
  void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                      const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                      const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void determine_torpedo_board_position(sub8_msgs::VisionRequest::Response &resp);
  bool request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                             sub8_msgs::VisionRequest::Response &resp);
  void segment_board(const cv::Mat &src, cv::Mat &dest);
  bool find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners);
  double image_proc_scale = 0.5;
private:
  ros::NodeHandle nh;
  ros::ServiceServer service;

  image_transport::CameraSubscriber left_image_sub, right_image_sub;
  image_transport::ImageTransport image_transport;
  image_geometry::PinholeCameraModel left_cam_model, right_cam_model;

  sub::Contour left_corners, right_corners;
  sub::ImageWithCameraInfo left_most_recent;
  sub::ImageWithCameraInfo right_most_recent;

  sub::RvizVisualizer rviz;
};