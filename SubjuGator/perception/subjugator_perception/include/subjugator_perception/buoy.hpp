#pragma once
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>  // TicToc
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <Eigen/StdVector>
#include <algorithm>
#include <iostream>
#include <map>
#include <mil_vision_lib/cv_tools.hpp>
#include <mil_vision_lib/pcl_tools.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "subjugator_msgs/VisionRequest.h"
#include "subjugator_msgs/VisionRequest2D.h"

// For stack-tracing on seg-fault
#define BACKWARD_HAS_BFD 1
#include <subjugator_build_tools/backward.hpp>

// ROS_NAMESPACE=/camera/front/left rosrun image_proc image_proc
// rosbag play ./holding_buoy_mil.bag -r 0.1
// [1]
// http://docs.ros.org/hydro/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html
// [2] http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html
/* TODO
  - Smarter buoy tracking
  - Better caching (instead of immediate assignment)
*/

class SubjuGatorBuoyDetector
{
public:
  SubjuGatorBuoyDetector();
  ~SubjuGatorBuoyDetector();
  void compute_loop(const ros::TimerEvent &);
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &);
  void image_callback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
  bool get_last_image(cv::Mat &last_image);
  bool determine_buoy_position(const image_geometry::PinholeCameraModel &cam_model, const std::string &target_color,
                               const cv::Mat &image_raw, const sub::PointCloudT::Ptr &point_cloud_raw,
                               Eigen::Vector3f &center);
  bool segment_buoy(cv::Mat &input_image, cv::Point &center, std::vector<sub::Contour> &output_contours,
                    std::string &target_name);
  bool request_buoy_position_2d(subjugator_msgs::VisionRequest2D::Request &req,
                                subjugator_msgs::VisionRequest2D::Response &resp);
  bool request_buoy_position(subjugator_msgs::VisionRequest::Request &req,
                             subjugator_msgs::VisionRequest::Response &resp);
  // Visualize
  int vp1;
  int vp2;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  sub::RvizVisualizer rviz;

  ros::Timer compute_timer;
  ros::Subscriber data_sub;
  ros::NodeHandle nh;
  ros::ServiceServer service_2d;
  ros::ServiceServer service_3d;
  double buoy_radius;

  std::map<std::string, sub::Range> color_ranges;
  Eigen::Vector3f last_bump_target;

  image_transport::CameraSubscriber image_sub;
  image_transport::ImageTransport image_transport;
  image_transport::Publisher image_pub;
  cv::Mat last_draw_image;

  image_geometry::PinholeCameraModel cam_model;

  ros::Time image_time;
  ros::Time last_cloud_time;
  sub::PointCloudT::Ptr current_cloud;
  sensor_msgs::ImageConstPtr last_image_msg;

  bool got_cloud, got_image, line_added, computing, need_new_cloud;
};
