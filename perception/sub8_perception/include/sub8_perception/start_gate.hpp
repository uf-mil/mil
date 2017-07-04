#pragma once
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <std_srvs/SetBool.h>
#include <sub8_msgs/VisionRequest.h>
#include <sub8_msgs/VisionRequest2D.h>

#include <sub8_vision_lib/stereo_base.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>

class Sub8StartGateDetector : public StereoBase
{
public:
  Sub8StartGateDetector();

  virtual std::vector<cv::Point> get_2d_feature_points(cv::Mat image);

  ros::NodeHandle nh;
  void run();

private:
  void determine_start_gate_position();

  // Some filtering params used by the 'process_image' function
  int canny_low_;
  float canny_ratio_;
  int blur_size_;
  int dilate_amount_;

  double get_angle(cv::Point a, cv::Point b, cv::Point c);
  // Helper function that checks if a contour is of a gate shape
  bool valid_contour(std::vector<cv::Point> &contour);
  // Helper function that does a blur and filters
  cv::Mat process_image(cv::Mat &image);
  // Given an array of contours, returns a polygon that is most similar to that of a gate
  std::vector<cv::Point> contour_to_2d_features(std::vector<std::vector<cv::Point>> &contour);
  // Given a set of points, find the center points between the closest point pairs
  std::vector<cv::Point> get_corner_center_points(const std::vector<cv::Point> &features);

  ros::Publisher marker_pub_;
  void visualize_k_gate_normal();
  void visualize_3d_points_rviz(const std::vector<Eigen::Vector3d> &feature_pts_3d);

  Eigen::Affine3d gate_pose_;
  bool gate_found_;
  ros::Time last_time_found_;
  ros::Duration timeout_for_found_;

  ros::ServiceServer active_service_;
  ros::ServiceServer vision_request_service_;
  bool set_active_enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool vision_request_cb(sub8_msgs::VisionRequest::Request &req, sub8_msgs::VisionRequest::Response &resp);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::TransformStamped transform_to_map_;
};
