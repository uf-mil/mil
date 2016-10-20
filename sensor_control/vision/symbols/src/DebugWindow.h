
#ifndef DEBUG_WINDOW_H
#define DEBUG_WINDOW_H

//#define DO_DEBUG

#ifdef DO_DEBUG

#include <fstream>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ShapeFind.h"
#include "navigator_msgs/DockShapes.h"

using namespace cv;
class DebugWindow {
 private:
  static Mat color_frame;
  static navigator_msgs::DockShapes symbols;

 public:
  static std::vector<navigator_msgs::DockShapes> allFoundSymbols;
  static void init();
  static void UpdateColor(Mat& frame);
  static void UpdateResults(navigator_msgs::DockShapes& s);
  static void DrawAll();
};

#endif

#endif
