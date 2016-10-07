#ifndef SHAPE_DETECTOR_H
#define SHAPE_DETECTOR_H
#include <ros/ros.h>
#include <numeric>
#include "opencv2/opencv.hpp"
//#define DO_SHAPE_DEBUG
class ShapeDetector {
 private:
  static float findAngle(cv::Point p1, cv::Point p2, cv::Point p3);
  static float chisquared(std::vector<float> observed, float expected);
  static float findVariance(std::vector<float> observed);

  static float CROSS_BOUNDING_AREA_LOW;
  static float CROSS_BOUNDING_AREA_HIGH;
  static float TRI_BOUNDING_AREA_LOW;
  static float TRI_BOUNDING_AREA_HIGH;
  static float CIRCLE_BOUNDING_AREA_LOW;
  static float CIRCLE_BOUNDING_AREA_HIGH;
  static float CROSS_MIN_AREA;
  static float TRI_MIN_AREA;
  static float CIRCLE_MIN_AREA;
  static float CROSS_ANGLE_VARIANCE;
  static float CROSS_ANGLE_CHI;

  static bool checkBoundingAreaCross(std::vector<cv::Point> &points);
  static bool checkBoundingAreaTriangle(std::vector<cv::Point> &points);
  static bool checkBoundingAreaCircle(std::vector<cv::Point> &points);

  static bool angleTestCross(std::vector<cv::Point> &points);
  static bool angleTestTriangle(std::vector<cv::Point> &points);
  static bool angleTestCirlce(std::vector<cv::Point> &points);

  static bool testRatioAreaPerimeterCircle(std::vector<cv::Point> &points);
  // static bool testRatioAreaPerimeterTriangle(std::vector<cv::Point> &points);
  static bool testRatioAreaPerimeterCross(std::vector<cv::Point> &points);
  static bool testPointAlignmentTriangle(std::vector<cv::Point> &points);

 public:
  static void init(ros::NodeHandle &nh);
  static bool isCross(std::vector<cv::Point> &points);
  static bool isTriangle(std::vector<cv::Point> &points);
  static bool isCircle(std::vector<cv::Point> &points);
};

#endif
