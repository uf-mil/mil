#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <mil_tools/mil_tools.hpp>
#include <mil_vision_lib/cv_tools.hpp>
#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/circular_buffer.hpp>

#include <visualization_msgs/Marker.h>

#include <sub8_msgs/VisionRequest.h>
#include <sub8_msgs/VisionRequest2D.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

class Sub8StartGateDetector
{
public:
  Sub8StartGateDetector();

private:
  bool set_active_enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool vision_request_cb(sub8_msgs::VisionRequest::Request &req, sub8_msgs::VisionRequest::Response &resp);

  void run();

  // Main algorithm that gets left and right image pointers and finds gate in 3D
  void determine_start_gate_position();
  // Helper function that returns the angle between 2 lines
  double get_angle(cv::Point a, cv::Point b, cv::Point c);
  // Helper function that checks if a contour is of a gate shape
  bool valid_contour(std::vector<cv::Point> &contour);
  // Helper function that does a blur and filters
  cv::Mat process_image(cv::Mat &image);

  // Given an array of contours, returns a polygon that is most similar to that of a gate
  std::vector<cv::Point> contour_to_2d_features(std::vector<std::vector<cv::Point>> &contour);
  // Given a set of points, find the center points between the closest point pairs
  std::vector<cv::Point> get_corner_center_points(const std::vector<cv::Point> &features);
  // Distance-based stereo matchin
  std::vector<int> shortest_pair_stereo_matching(const std::vector<cv::Point> &features_l,
                                                 const std::vector<cv::Point> &features_r, int y_axis_diff_thresh = 0);
  // Finds a plane
  std::vector<double> best_fit_plane_standard(const std::vector<Eigen::Vector3d> &feature_pts_3d);

  // Some visualization
  void visualize_3d_reconstruction(const std::vector<Eigen::Vector3d> &feature_pts_3d, cv::Matx34d left_cam_mat,
                                   cv::Matx34d right_cam_mat, cv::Mat &current_left_right,
                                   cv::Mat &current_image_right);
  void visualize_3d_points_rviz(const std::vector<Eigen::Vector3d> &feature_pts_3d,
                                const std::vector<Eigen::Vector3d> &proj_pts);
  void visualize_k_gate_normal();

  ros::NodeHandle nh;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher debug_image_pub_left_;
  image_transport::Publisher debug_image_pub_right_;
  image_transport::Publisher debug_image_pub_canny_;
  mil_vision::ROSCameraStream<cv::Vec3b> left_cam_stream_;
  mil_vision::ROSCameraStream<cv::Vec3b> right_cam_stream_;

  // To prevent invalid img pointers from being passed to toCvCopy (segfault)
  boost::mutex left_mtx_, right_mtx_;

  // Should there be processing to find the gate
  bool active_;

  // How far off in time can left and right cameras be off
  double sync_thresh_;

  // Publish marker visualization as well as the normal approximation
  ros::Publisher marker_pub_;

  // Toggle to run the vision
  ros::ServiceServer active_service_;
  ros::ServiceServer vision_request_service_;

  // Some filtering params used by the 'process_image' function
  int canny_low_;
  float canny_ratio_;
  int blur_size_;
  int dilate_amount_;

  // how often to process images
  double dt_;

  Eigen::Vector3d gate_position_;
  Eigen::Quaterniond gate_orientation_;
  bool gate_found_;
  ros::Time last_time_found_;
  ros::Duration timeout_for_found_;

  int n_states_;
  int n_measurements_;
  cv::KalmanFilter k_filter_;
  void init_kalman_filter();
  void update_kalman_filter(Eigen::Vector3d center_point, Eigen::Vector3d normal_vector);
  cv::Mat get_measurement_as_cv_mat(Eigen::Vector3d center_point, Eigen::Vector3d normal_vector);
  void reset_filter_from_time();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::TransformStamped transform_to_map_;
};