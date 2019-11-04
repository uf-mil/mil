#pragma once

#include <rosbag/view.h>
#include <QApplication>

#include <opencv2/core.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>

#include <map>
#include <memory>

#include <point_cloud_object_detection_and_recognition/pcodar_types.hpp>

#include <mil_msgs/PerceptionObjectArray.h>

// namespace pcod
// {
// namespace label_tool
// {

struct course_object
{
  mil_msgs::PerceptionObject p_object;
  std::vector<Eigen::Vector4d> bounding_box_points;
  bool visualize = true;
  cv::Scalar color;
};

using id_to_labeled_object = std::map<uint16_t, course_object>;

class label_model
{
public:
  label_model(int argc, char* argv[], std::shared_ptr<id_to_labeled_object> object_map_ptr);
  bool has_next_image();
  cv::Mat get_next_image();
  void generate_bag();
  void restart();
  void populate_map();

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<rosbag::View> v_;

  std::string input_bag_name_;
  std::string output_bag_name_;

  Eigen::Matrix<double, 3, 4> intrinsics_;
  Eigen::Affine3d e_baselink_to_stereoright_;
  Eigen::Affine3d e_velodyne_to_baselink_;

  rosbag::Bag bag_;

  pcodar::point_cloud mega_cloud_;

  rosbag::View::iterator bag_iter_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  sensor_msgs::ImageConstPtr last_image_;

  std::shared_ptr<id_to_labeled_object> object_map_ptr_;
};

// }  // namespace label_tool
// }  // namespace pcod
