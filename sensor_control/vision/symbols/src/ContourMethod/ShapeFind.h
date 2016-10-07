#ifndef SHAPE_FIND_HPP
#define SHAPE_FIND_HPP
// Finds shapes in Binary frame

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ShapeDetector.h"
#include "geometry_msgs/Point.h"
#include "navigator_msgs/DockShapes.h"
using namespace cv;
class ShapeFind {
 public:
  ShapeFind(std::string color);
  void GetSymbols(Mat frame, navigator_msgs::DockShapes* symbols);

 private:
  std::string parseColor;
  Mat binary_frame;
  std::vector<std::vector<Point> > contoursfindMat;
  std::vector<Vec4i> hierarchyfindMat;
  std::vector<std::vector<cv::Point> > shapes;
  Point findCenter(std::vector<Point> points);
  navigator_msgs::DockShape fillSymbolMsg(std::vector<cv::Point> points,
                                          std::string symbol);
  void FindContours();
  void FindShapes();
};

#endif
