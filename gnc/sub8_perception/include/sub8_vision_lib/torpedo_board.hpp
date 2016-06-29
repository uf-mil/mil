#pragma once
#include <string>
#include <vector>
#include <utility>  // std::pair
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

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

#include <sub8_vision_lib/pcl_tools.hpp>
#include <sub8_vision_lib/cv_tools.hpp>
#include <sub8_vision_lib/torpedo_board.hpp>
#include <sub8_msgs/TorpBoardPoseRequest.h>
#include <sub8_msgs/TBDetectionSwitch.h>

// #define SEGMENTATION_DEBUG

#ifdef SEGMENTATION_DEBUG
#warning(Compiling with segmentation debugging info enabled)
#else
#warning(Compiling with NO segmentation debugging info)
#endif

/*
  Warning:
  This class cannot be copy constructed. 
  For examlple, the following will not compile:
    Sub8TorpedoBoardDetector tb_detector = Sub8TorpedoBoardDetector();
  Do this instead:
    Sub8TorpedoBoardDetector tb_detector();
*/

class Sub8TorpedoBoardDetector {

public:
  Sub8TorpedoBoardDetector();
  ~Sub8TorpedoBoardDetector();

  // Public Variables
  double image_proc_scale;
  sub::Contour left_corners, right_corners;
  sub::ImageWithCameraInfo left_most_recent, right_most_recent;

private:
  // Callbacks
  bool detection_activation_switch(
      sub8_msgs::TBDetectionSwitch::Request &req,
      sub8_msgs::TBDetectionSwitch::Response &resp);
  void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                           const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void
  right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                       const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);

  // Detection / Processing
  void run();
  void determine_torpedo_board_position();
  void segment_board(const cv::Mat &src, cv::Mat &dest, cv::Mat &dbg_img,
                     bool draw_dbg_img = false);
  bool find_board_corners(const cv::Mat &segmented_board, std::vector<cv::Point> &corners,
                          bool draw_dbg_left = true);

  // ROS
  ros::NodeHandle nh;
  ros::ServiceServer detection_switch;
  ros::ServiceClient pose_client;
  image_transport::CameraSubscriber left_image_sub, right_image_sub;
  image_transport::ImageTransport image_transport;
  image_transport::Publisher debug_image_pub;
  image_geometry::PinholeCameraModel left_cam_model, right_cam_model;

  // Torpedo Board detection will be attempted when true
  bool active;

  // To prevent invalid img pointers from being passed to toCvCopy (segfault)
  boost::mutex left_mtx, right_mtx;

// Frames will be considered synchronized if their stamp difference is less than
// this (in seconds)
#if __cplusplus > 199711L
  static constexpr double sync_thresh = 0.25;
#else
  static const double sync_thresh = 0.25;
#endif

  // Goes into sequential id for pos_est srv request
  long long int run_id;

  // RVIZ
  sub::RvizVisualizer rviz;

  // DBG images will be generated and published when true
  bool generate_dbg_img;
  cv::Mat debug_image;
  cv::Rect upper_left, upper_right, lower_left, lower_right;
};

class TorpedoBoardReprojectionCost {
public:
  TorpedoBoardReprojectionCost(cv::Matx34d &proj_L, cv::Matx34d &proj_R,
                               std::vector<cv::Point> &corners_L,
                               std::vector<cv::Point> &corners_R);
  ~TorpedoBoardReprojectionCost();

  template <typename T>
  bool operator()(const T *const x, const T *const y, const T *const z,
                  const T *const yaw, T *residual) const;

private:
  static std::vector<cv::Point> getProjectedCorners(double center_x,
                                                    double center_y,
                                                    double center_z, double yaw,
                                                    cv::Matx34d &proj_matrix);

#if __cplusplus > 199711L
  static constexpr double height_m = 1.24; // in meters, aka(49 in.)
  static constexpr double width_m = 0.61;  // in meters, aka(24 in.)
#else
  static const double height_m = 1.24; // in meters, aka(49 in.)
  static const double width_m = 0.61;  // in meters, aka(24 in.)
#endif

  const cv::Matx34d proj_L;
  const cv::Matx34d proj_R;

  const std::vector<cv::Point> img_corners_L;
  const std::vector<cv::Point> img_corners_R;
};
