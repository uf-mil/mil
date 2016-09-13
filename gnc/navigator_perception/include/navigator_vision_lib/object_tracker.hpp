#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <navigator_vision_lib/cv_tools.hpp>

class ObjectTracker{
public:
  void begin_tracking_object(std::vector<cv::Point> points, cv::Mat left_frame);
  bool track_object(cv::Mat left_frame, std::vector<cv::Point2f>& points_2d);
  void clear();

private:
  std::vector<cv::Point2f> prev_points;
  int template_size = 20;
  int search_size = 40;

  cv::Mat getROI(cv::Mat image, int point_x, int point_y, int size, cv::Rect& roi);



  //DEBUG

  cv::Mat orig_frame;
  int count = 0;
};

int get_furthest_point(std::vector<cv::Point2f> mymodel, int point);

