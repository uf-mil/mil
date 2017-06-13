#pragma once

#include <vector>
#include <iostream>
#include <string>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <sub8_msgs/TBDetectionSwitch.h>
#include <sub8_msgs/TorpBoardPoseRequest.h>
#include <mil_vision_lib/cv_tools.hpp>
#include <sub8_vision_lib/visualization.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/circular_buffer.hpp>

#include <visualization_msgs/Marker.h>

// #include <pcl/features/normal_3d.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>

class Sub8StartGateDetector
{
public:
  Sub8StartGateDetector();
  mil_vision::ImageWithCameraInfo left_most_recent, right_most_recent;

private:
  
  // Combinations of k elements from a set of size n (indexes)
  void combinations(uint8_t n, uint8_t k, std::vector<std::vector<uint8_t> > &idx_array);
  void _increase_elements_after_level(std::vector<uint8_t> comb, std::vector<std::vector<uint8_t> > &comb_array,
                                      uint8_t n, uint8_t k, uint8_t level);

  void left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                           const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                            const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
  void run();
  void determine_start_gate_position();

  double get_angle(cv::Point a, cv::Point b, cv::Point c);

  bool valid_contour(std::vector<cv::Point> &contour);

  ros::NodeHandle nh;
  image_transport::CameraSubscriber left_image_sub, right_image_sub;
  image_transport::ImageTransport image_transport;
  image_transport::Publisher debug_image_pub_left;
  image_transport::Publisher debug_image_pub_right;
  image_transport::Publisher debug_image_pub_canny;

  image_geometry::PinholeCameraModel left_cam_model, right_cam_model;

  bool active;
  double sync_thresh;

  // To prevent invalid img pointers from being passed to toCvCopy (segfault)
  boost::mutex left_mtx, right_mtx;
  cv::KalmanFilter kf;

  boost::circular_buffer<Eigen::Vector3d> center_estimate_xyz;
  ros::Publisher marker_pub;

  ros::Publisher center_gate_pub;
  ros::Publisher normal_gate_pub;

  int canny_low;
  float canny_ratio;
  int blur_size;
  int dilate_amount;

};