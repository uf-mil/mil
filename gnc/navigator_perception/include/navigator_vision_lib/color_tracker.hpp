#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <math.h>
#include <navigator_vision_lib/cv_tools.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <navigator_msgs/ScanTheCodeMission.h>


class ColorTracker{
public:
  bool track(cv::Mat frame_left, std::vector<cv::Point2f> points, double image_proc_scale);
  void clear();
  void set_status(bool i);

private:
  bool status = 0;
  bool mission_complete = 0;
  bool turned_black = 0;
  int colors_found = 0;
  bool started = 0;
  int prev_color = 0;

  cv::Scalar last_color;
  ros::NodeHandle nh;
  ros::ServiceServer send_status = nh.advertiseService("scan_the_code/mission_status", &ColorTracker::check_status, this);
  image_transport::ImageTransport image_transport = image_transport::ImageTransport(nh);

  image_transport::Publisher debug_image_color = image_transport.advertise("stereo_model_fitter/debug_img/color", 1, true);

  bool check_status(
      navigator_msgs::ScanTheCodeMission::Request &req,
      navigator_msgs::ScanTheCodeMission::Response &resp);

  std::string colors[3];


  cv::Scalar red = cv::Scalar(0,0,1,0);
  cv::Scalar blue = cv::Scalar(1,0,0,0);
  cv::Scalar yellow = cv::Scalar(1,0,1,0);
  cv::Scalar green = cv::Scalar(0,1,0,0);


};

