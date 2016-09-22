#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <navigator_vision_lib/cv_tools.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ObjectTracker{
public:
  void begin_tracking_object(std::vector<cv::Point> points, cv::Mat left_frame);
  bool track_object(cv::Mat left_frame, std::vector<cv::Point2f>& points_2d);
  void clear();

private:
  std::vector<cv::Point2f> prev_points;
  cv::Mat orig_frame;

  ros::NodeHandle nh;
  image_transport::ImageTransport image_transport = image_transport::ImageTransport(nh);
  image_transport::Publisher debug_image_tracking_points = image_transport.advertise("stereo_model_fitter/debug_img/tracking_points", 1, true);


  cv::Mat getROI(cv::Mat image, int point_x, int point_y, int size, cv::Rect& roi);



  //DEBUG

  int count = 0;
};

int get_furthest_point(std::vector<cv::Point2f> mymodel, int point);

