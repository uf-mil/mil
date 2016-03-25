#pragma once
#include <string>
#include <vector>
#include <iostream>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>
#include <sub8_pcl/torpedo_board.hpp>

#include "ros/ros.h"

#include "sub8_msgs/VisionRequest.h"

class Sub8TorpedoBoardDetector {
public:
  Sub8TorpedoBoardDetector();
  ~Sub8TorpedoBoardDetector();
  void image_callback(const sensor_msgs::ImageConstPtr &image_msg,
                      const sensor_msgs::CameraInfoConstPtr &info_msg);
  void determine_torpedo_board_position(const image_geometry::PinholeCameraModel &cam_model,
                               const cv::Mat &image_raw);
  bool request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                             sub8_msgs::VisionRequest::Response &resp);
  void board_segmentation(const cv::Mat &src, cv::Mat &dest);

  void find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners);
  
  sub::FrameHistory frame_history;

  ros::NodeHandle nh;

  image_transport::CameraSubscriber image_sub;  // ?? gcc complains if I remove parentheses
  image_transport::ImageTransport image_transport;
  image_geometry::PinholeCameraModel cam_model;

};