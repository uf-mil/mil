#pragma once
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <sub8_msgs/TBDetectionSwitch.h>
#include <sub8_msgs/TorpBoardPoseRequest.h>
#include <mil_vision_lib/cv_tools.hpp>
#include <sub8_vision_lib/visualization.hpp>

#include <mil_tools/mil_tools.hpp>

// #define SEGMENTATION_DEBUG

/*
  Warning:
  Because of its multithreadedness, this class cannot be copy constructed.
  For examlple, the following will not compile:
    Sub8TorpedoBoardDetector tb_detector = Sub8TorpedoBoardDetector();
  Do this instead:
    Sub8TorpedoBoardDetector tb_detector();
*/

class Sub8TorpedoBoardDetector
{
public:
  Sub8TorpedoBoardDetector();
  ~Sub8TorpedoBoardDetector();

  // Public Variables
  double image_proc_scale, feature_min_distance;
  int diffusion_time, max_features, feature_block_size;
  mil_vision::Contour left_corners, right_corners;
  mil_vision::ImageWithCameraInfo left_most_recent, right_most_recent;

private:
  // Callbacks
  bool detection_activation_switch(sub8_msgs::TBDetectionSwitch::Request &req,
                                   sub8_msgs::TBDetectionSwitch::Response &resp);
  void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                           const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                            const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);

  // Detection / Processing
  void run();
  void determine_torpedo_board_position();
  void segment_board(const cv::Mat &src, cv::Mat &dest, cv::Mat &dbg_img, bool draw_dbg_img = false);
  bool find_board_corners(const cv::Mat &segmented_board, std::vector<cv::Point> &corners, bool draw_dbg_left = true);
  void stereo_correspondence(const cv::Mat &gray_L, const cv::Mat &gray_R, const std::vector<cv::Point> &features_L,
                             const std::vector<cv::Point> &features_R,
                             std::vector<std::vector<int> > &corresponding_feat_idxs);

  // std::vector<cv::Point2d> project_rotated_model(Eigen::Matrix<double, 3, 4> cam_matx,
  //                                                Eigen::Quaterniond orientation);

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
  static constexpr double sync_thresh = 0.5;
#else
  static const double sync_thresh = 0.5;
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

class TorpedoBoardReprojectionCost
{
public:
  TorpedoBoardReprojectionCost(cv::Matx34d &proj_L, cv::Matx34d &proj_R, std::vector<cv::Point> &corners_L,
                               std::vector<cv::Point> &corners_R);
  ~TorpedoBoardReprojectionCost();

  template <typename T>
  bool operator()(const T *const x, const T *const y, const T *const z, const T *const yaw, T *residual) const;

private:
  static std::vector<cv::Point> getProjectedCorners(double center_x, double center_y, double center_z, double yaw,
                                                    cv::Matx34d &proj_matrix);

#if __cplusplus > 199711L
  static constexpr double height_m = 1.24;  // in meters, aka(49 in.)
  static constexpr double width_m = 0.61;   // in meters, aka(24 in.)
#else
  static const double height_m = 1.24;  // in meters, aka(49 in.)
  static const double width_m = 0.61;   // in meters, aka(24 in.)
#endif

  const cv::Matx34d proj_L;
  const cv::Matx34d proj_R;

  const std::vector<cv::Point> img_corners_L;
  const std::vector<cv::Point> img_corners_R;
};

/*
  Helper Functions
*/
// Edge preserving image denoising
void anisotropic_diffusion(const cv::Mat &src, cv::Mat &dest, int t_max);

// Pick a plane from a triplet of 3 points from a vector of points
void best_plane_from_combination(const std::vector<Eigen::Vector3d> &point_list, double distance_threshold,
                                 std::vector<double> &result_coeffs);

// Calculate coefficients of plane equation from 3 points
void calc_plane_coeffs(Eigen::Vector3d &pt1, Eigen::Vector3d &pt2, Eigen::Vector3d &pt3,
                       std::vector<double> &plane_coeffs);

// Calculate distance of point to plane defined by coefficients of the plane equation of the
// form a*x + b*y + c*z + d = 0
double point_to_plane_distance(double a, double b, double c, double d, Eigen::Vector3d pt);
