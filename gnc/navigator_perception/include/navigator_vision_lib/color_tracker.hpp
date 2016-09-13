#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <navigator_vision_lib/cv_tools.hpp>

class ColorTracker{
public:
  bool track(cv::Mat frame_left, std::vector<cv::Point2f> points, double image_proc_scale, std::vector<char>& colors);
  void clear();

private:
};

