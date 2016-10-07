#pragma once
#include <navigator_msgs/DockShapes.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <memory>
#include "../DockShapeVision.h"
#include "opencv2/opencv.hpp"
using namespace cv;

//#define DO_DEBUG
#define DO_ROS_DEBUG
#ifdef DO_ROS_DEBUG
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#endif
class GrayscaleContour : public DockShapeVision {
 private:
  Mat colorFrame, croppedFrame, grayscaleFrame, edgesFrame;
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > shapes;
  Rect roi;

  void CropFrame();
  void ConvertToGrayscale();
  void DetectEdges();
  void FindContours();
  bool GetColor(int shapeIndex, std::string& color);
  Point findCenter(std::vector<Point>& points);
  Mat contoursFrame;
  int frame_height;
  int frame_width;

  bool filterArea(std::vector<Point> contour);

  bool isTriangle(std::vector<Point>& points);
  bool isCross(std::vector<Point>& points);
  bool isCircle(std::vector<Point>& points);

  void setShapePoints(navigator_msgs::DockShape& dockShape,
                      std::vector<Point>& points);

  static double findAngle(cv::Point& p1, cv::Point& p2, cv::Point& p3);
  static void findAngles(std::vector<Point>& points,
                         std::vector<double>& angles);
#ifdef DO_ROS_DEBUG
  std::unique_ptr<image_transport::ImageTransport> image_transport;
  image_transport::Publisher color_debug_publisher;
  image_transport::Publisher contour_debug_publisher;
#endif

  // Constants to use ros params for
  struct CannyParams {
    int thresh1;
    int thresh2;
  };
  CannyParams cannyParams;
  double minArea;
  int blur_size;
  double triangleEpsilon;
  double crossEpsilon;
  double triRectErrorThreshold;
  double triSideErrorThreshold;
  double triAngleMeanErrorThreshold;
  double triAngleVarErrorThreshold;
  double crossRectErrorThreshold;
  double crossSideErrorThreshold;
  double crossAngleMeanErrorThreshold;
  double crossAngleVarErrorThreshold;
  double circleEnclosingErrorThreshold;

 public:
  GrayscaleContour(ros::NodeHandle& nh);
  void GetShapes(cv::Mat& frame, cv::Rect roi,
                 navigator_msgs::DockShapes& symbols);
  void init();
};
