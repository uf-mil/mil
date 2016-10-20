// Grabs frame from ROS, erode/dilate, converts to HSV, produces Red Green Blue
// Black binary frames;
#ifndef FROM_PROC_H
#define FROM_PROC_H

#include <ros/ros.h>
#include "../DebugWindow.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

class FrameProc {
 private:
  static int blur_size;
  struct ColorThresh {
    Scalar low;
    Scalar high;
  };
  ColorThresh red;
  ColorThresh red2;
  ColorThresh blue;
  ColorThresh green;

  Mat rgb_frame;
  Mat hsv_frame;
  Mat binary_blue_frame;
  Mat binary_red_frame;
  Mat binary_green_frame;

  void ErodeDilate();
  void ConvertHSV();
  void ThresholdColors();

 public:
  FrameProc();
  void init(ros::NodeHandle& nh);
  void Prepare(Mat& frame);
  Mat GetRed();
  Mat GetBlue();
  Mat GetGreen();
};

#endif
