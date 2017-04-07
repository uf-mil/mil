#pragma once
#include <string>
#include <iostream>

#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <std_srvs/SetBool.h>

#include "sub8_msgs/VisionRequest2D.h"
#include "sub8_msgs/BMatrix.h"

using namespace boost::accumulators;

class Sub8StartGateDetector
{
public:
  Sub8StartGateDetector();
  ~Sub8StartGateDetector();

  // 76 inches x 39.5 inches
  //1.9304 x 1.0033
  const double START_GATE_WIDTH = 1.9304;  // Meters
  const double START_GATE_HEIGHT = 1.0033;
  const double START_GATE_SIZE_TOLERANCE = 10000;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
  bool requestStartGatePosition2d(sub8_msgs::VisionRequest2D::Request &req,
                                      sub8_msgs::VisionRequest2D::Response &resp);
  bool requestStartGateEnable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool requestStartGateDistance(sub8_msgs::BMatrix::Request &req,
                                                     	sub8_msgs::BMatrix::Response &resp);
  void findGate(const sensor_msgs::ImageConstPtr &image_msg);

  double getGateDistance();

  ros::NodeHandle nh_;
  bool running_;

  image_transport::ImageTransport image_transport_;
  image_transport::CameraSubscriber image_sub_;

  image_geometry::PinholeCameraModel cam_model_;
  ros::Time image_time_;

  int rows_;
  int cols_;

  boost::circular_buffer<std::vector<std::vector<cv::Point2f>>> gate_line_buffer_;
  accumulator_set<int, features<tag::mean, tag::variance>> accX_;
  accumulator_set<int, features<tag::mean, tag::variance>> accY_;
  accumulator_set<int, features<tag::mean, tag::variance>> accSizeX_;
  accumulator_set<int, features<tag::mean, tag::variance>> accSizeY_;

  ros::ServiceServer service_enable_;
  ros::ServiceServer service_2d_;
  ros::ServiceServer service_distance_;
};