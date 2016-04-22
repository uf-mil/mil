#pragma once
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <sub8_pcl/pcl_tools.hpp>
#include <sub8_pcl/cv_tools.hpp>
#include <sub8_pcl/torpedo_board.hpp>
#include <sub8_msgs/VisionRequest.h>

// #include <ceres/ceres.h>
// #include <glog/logging.h>

class Sub8TorpedoBoardDetector {
public:
  Sub8TorpedoBoardDetector(double im_proc_scale = 0, bool gen_dbg_img = true, std::string l_img_topic = "", std::string r_img_topic = "", std::string srv_name = "",
                           std::string viz_topic = "");
  ~Sub8TorpedoBoardDetector();
  void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                      const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                      const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void determine_torpedo_board_position(sub8_msgs::VisionRequest::Response &resp);
  bool request_torpedo_board_position(sub8_msgs::VisionRequest::Request &req,
                             sub8_msgs::VisionRequest::Response &resp);
  void segment_board(const cv::Mat &src, cv::Mat &dest, cv::Mat &dbg_img, bool draw_dbg_img = false);
  bool find_board_corners(const cv::Mat &segmented_board, sub::Contour &corners, bool draw_dbg_left = true);
  double image_proc_scale;
private:
  ros::NodeHandle nh;
  ros::ServiceServer service;

  image_transport::CameraSubscriber left_image_sub, right_image_sub;
  image_transport::ImageTransport image_transport;
  image_transport::Publisher debug_image_pub;
  image_geometry::PinholeCameraModel left_cam_model, right_cam_model;

  sub::Contour left_corners, right_corners;
  sub::ImageWithCameraInfo left_most_recent;
  sub::ImageWithCameraInfo right_most_recent;

  static const double sync_thresh = 0.25;

  sub::RvizVisualizer rviz;

  cv::Mat debug_image;
  cv::Rect upper_left, upper_right, lower_left, lower_right;
  const bool generate_dbg_img;
};

class TorpedoBoardReprojectionCost {
public:
  TorpedoBoardReprojectionCost(cv::Matx34d &proj_L, cv::Matx34d &proj_R, std::vector<cv::Point> &corners_L, std::vector<cv::Point> &corners_R);
  ~TorpedoBoardReprojectionCost();
  
  template <typename T> bool operator() (const T* const x, const T* const y, 
               const T* const z, const T* const yaw, T* residual) const;

  

private:

  static std::vector<cv::Point> getProjectedCorners(double center_x, double center_y, double center_z, double yaw, cv::Matx34d &proj_matrix);
  
  static const double height_m = 1.24; // in meters, aka(49 in.)
  static const double width_m = 0.61;  // in meters, aka(24 in.)

  const cv::Matx34d proj_L;
  const cv::Matx34d proj_R;

  const std::vector<cv::Point> img_corners_L;
  const std::vector<cv::Point> img_corners_R;
};