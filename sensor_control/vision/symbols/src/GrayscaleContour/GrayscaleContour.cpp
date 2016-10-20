#include "GrayscaleContour.h"
#include <sstream>
GrayscaleContour::GrayscaleContour(ros::NodeHandle& nh) : DockShapeVision(nh) {
  // Set ros params
  nh.param<int>("grayscale/canny/thresh1", cannyParams.thresh1, 60);
  nh.param<int>("grayscale/canny/thresh2", cannyParams.thresh2, 100);
  nh.param<int>("grayscale/blur_size", blur_size, 7);
  nh.param<double>("grayscale/min_contour_area", minArea,
                   100.0 / (644.0 * 482.0));
  nh.param<double>("grayscale/triangle/epsilon", triangleEpsilon, 0.1);
  nh.param<double>("grayscale/cross/epsilon", crossEpsilon, 0.03);
  nh.param<double>("grayscale/triangle/rect_error_threshold",
                   triRectErrorThreshold, 0.1);
  nh.param<double>("grayscale/triangle/side_error_threshold",
                   triSideErrorThreshold, 0.1);
  nh.param<double>("grayscale/triangle/angle_mean_error_threshold",
                   triAngleMeanErrorThreshold, 5);
  nh.param<double>("grayscale/triangle/angle_var_error_threshold",
                   triAngleVarErrorThreshold, 10);
  nh.param<double>("grayscale/cross/rect_error_threshold",
                   crossRectErrorThreshold, 0.2);
  nh.param<double>("grayscale/cross/side_error_threshold",
                   crossSideErrorThreshold, 0.2);
  nh.param<double>("grayscale/cross/angle_mean_error_threshold",
                   crossAngleMeanErrorThreshold, 5);
  nh.param<double>("grayscale/cross/angle_var_error_threshold",
                   crossAngleVarErrorThreshold, 5);
  nh.param<double>("grayscale/circle/enclosing_circle_error_threshold",
                   circleEnclosingErrorThreshold, 0.2);

#ifdef DO_DEBUG
  namedWindow("Menu", CV_WINDOW_AUTOSIZE);
  namedWindow("Result", CV_WINDOW_AUTOSIZE);
  namedWindow("Grayscale", CV_WINDOW_AUTOSIZE);
  namedWindow("Contours", CV_WINDOW_AUTOSIZE);
  namedWindow("Edges", CV_WINDOW_AUTOSIZE);
#endif
#ifdef DO_ROS_DEBUG
  image_transport.reset(new image_transport::ImageTransport(nh));
  color_debug_publisher =
      image_transport->advertise("/dock_shapes/finder/debug_color", 1);
  contour_debug_publisher =
      image_transport->advertise("/dock_shapes/finder/debug_contours", 1);
#endif
}
void GrayscaleContour::init() {
#ifdef DO_DEBUG
  createTrackbar("thresh1", "Menu", &cannyParams.thresh1, 500);
  createTrackbar("thresh2", "Menu", &cannyParams.thresh2, 255);
#endif
}
void GrayscaleContour::GetShapes(cv::Mat& frame, cv::Rect roi,
                                 navigator_msgs::DockShapes& symbols) {
  if (frame.empty()) return;

  this->roi = roi;
  frame_width = frame.cols;
  frame_height = frame.rows;

  colorFrame = frame;
  CropFrame();
  Mat temp = croppedFrame.clone();
  bilateralFilter(temp, croppedFrame, blur_size, blur_size * 2, blur_size / 2);
  ConvertToGrayscale();
  DetectEdges();
  FindContours();

#ifdef DO_DEBUG
  Mat result = croppedFrame.clone();
#endif
#ifdef DO_ROS_DEBUG
  cv_bridge::CvImage ros_color_debug;
  ros_color_debug.encoding = "bgr8";
  ros_color_debug.image = croppedFrame.clone();
#endif

  for (int i = 0; i < contours.size(); i++) {
    navigator_msgs::DockShape dockShape;
    std::vector<cv::Point> approx_tri;
    std::vector<cv::Point> approx_cross;
    approxPolyDP(Mat(contours[i]), approx_tri,
                 arcLength(contours[i], true) * triangleEpsilon, true);
    approxPolyDP(Mat(contours[i]), approx_cross,
                 arcLength(contours[i], true) * crossEpsilon, true);
    if (isTriangle(approx_tri)) {
      dockShape.Shape = navigator_msgs::DockShape::TRIANGLE;
      setShapePoints(dockShape, approx_tri);
    } else if (isCross(approx_cross)) {
      dockShape.Shape = navigator_msgs::DockShape::CROSS;
      setShapePoints(dockShape, approx_cross);
    } else if (isCircle(contours[i])) {
      dockShape.Shape = navigator_msgs::DockShape::CIRCLE;
      setShapePoints(dockShape, contours[i]);
    } else
      continue;

    if (!GetColor(i, dockShape.Color)) continue;

#ifdef DO_DEBUG
    drawContours(result, contours, i, Scalar(0, 0, 255));
#endif
#ifdef DO_ROS_DEBUG
    drawContours(ros_color_debug.image, contours, i, Scalar(0, 0, 255));
#endif

    symbols.list.push_back(dockShape);
  }

#ifdef DO_DEBUG
  for (auto symbol : symbols.list) {
    putText(result, symbol.Shape + "\n(" + symbol.Color + ")",
            Point(symbol.CenterX + 10 - roi.x, symbol.CenterY - roi.y), 1, 1,
            Scalar(0, 0, 0), 3);
  }
  imshow("Result", result);
  imshow("Grayscale", grayscaleFrame);
  imshow("Edges", edgesFrame);
  imshow("Contours", contoursFrame);
  waitKey(5);
#endif
#ifdef DO_ROS_DEBUG
  for (auto symbol : symbols.list) {
    putText(ros_color_debug.image, symbol.Shape + "\n(" + symbol.Color + ")",
            Point(symbol.CenterX + 10 - roi.x, symbol.CenterY - roi.y), 1, 1,
            Scalar(0, 0, 0), 3);
  }
  color_debug_publisher.publish(ros_color_debug.toImageMsg());
  cv_bridge::CvImage ros_contours_debug;
  ros_color_debug.encoding = "mono8";
  ros_color_debug.image = contoursFrame.clone();
  contour_debug_publisher.publish(ros_color_debug.toImageMsg());
#endif
}
void GrayscaleContour::CropFrame() { croppedFrame = colorFrame(roi); }
void GrayscaleContour::ConvertToGrayscale() {
  cvtColor(croppedFrame, grayscaleFrame, CV_BGR2GRAY);
}
void GrayscaleContour::DetectEdges() {
  Canny(grayscaleFrame, edgesFrame, cannyParams.thresh1, cannyParams.thresh2);
}
void GrayscaleContour::FindContours() {
  contours.clear();
  hierarchy.clear();
  findContours(edgesFrame, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  // Filter out very small contours
  auto cit = contours.begin();
  auto hit = hierarchy.begin();
  while (cit != contours.end()) {
    if (filterArea(*cit)) {
      cit = contours.erase(cit);
      hit = hierarchy.erase(hit);
    } else {
      cit++;
      hit++;
    }
  }

#if defined(DO_DEBUG) || defined(DO_ROS_DEBUG)
  contoursFrame = Mat(edgesFrame.size(), edgesFrame.type(), Scalar(0, 0, 0));
  for (int i = 0; i < contours.size(); i++) {
    drawContours(contoursFrame, contours, i, Scalar(255, 255, 255));
  }
#endif
}
bool GrayscaleContour::GetColor(int Index, std::string& color) {
  Mat mask = Mat::zeros(croppedFrame.rows, croppedFrame.cols, CV_8UC1);
  drawContours(mask, contours, Index, Scalar(255), CV_FILLED);
  Scalar meanColor = mean(croppedFrame, mask);
  // std::cout << meanColor << std::endl;
  double sum = meanColor.val[0] + meanColor.val[1] + meanColor.val[2];
  // ~if (sum > 500) return false;
  double max = 0;
  if (meanColor.val[0] > max) {
    max = meanColor.val[0];
    color = navigator_msgs::DockShape::BLUE;
  }
  if (meanColor.val[1] > max) {
    max = meanColor.val[1];
    color = navigator_msgs::DockShape::GREEN;
  }
  if (meanColor.val[2] > max) {
    max = meanColor.val[2];
    color = navigator_msgs::DockShape::RED;
  }
  return true;
}
bool GrayscaleContour::isTriangle(std::vector<Point>& points) {
  // If contour is not 3 sided, is not a triangle
  if (points.size() != 3) return false;

  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);
  double expected_area, expected_perimeter, error;

  // Draw bounding rect, find area of triangle with this height/width, compare
  // to real area
  cv::Rect boundingRect = cv::boundingRect(points);
  expected_area = 0.5 * boundingRect.width * boundingRect.height;
  error = fabs(contour_area / expected_area - 1.0);
  if (error > triRectErrorThreshold) return false;

  // Find side length based on bounding rect width, compare to real perimeter of
  // contour
  expected_perimeter = boundingRect.width * 3;
  error = fabs(perimeter / expected_perimeter - 1.0);
  if (error > triSideErrorThreshold) return false;

  // Find angles (in degrees) of contour, compare to expected of 60 degrees with
  // low variance
  std::vector<double> angles;
  findAngles(points, angles);
  using namespace boost::accumulators;
  accumulator_set<int, features<tag::mean, tag::variance>> acc;
  for (double angle : angles) acc(angle);
  auto mean = boost::accumulators::mean(acc);
  auto variance = sqrt(boost::accumulators::variance(acc));
  if (fabs(mean - 60.0) > triAngleMeanErrorThreshold) return false;
  if (variance > triAngleVarErrorThreshold) return false;

  return true;
}
bool GrayscaleContour::isCross(std::vector<Point>& points) {
  // If polygon is not 12 sided, is not a cross
  if (points.size() != 12) return false;

  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);
  double expected_area, error;

  // Find length of 1 side using perimter/12, then compare area of contour with
  // this side length to real area
  double side_length = perimeter / 12.0;
  expected_area = 5.0 * pow(side_length, 2);
  error = fabs(contour_area / expected_area - 1.0);
  if (error > crossRectErrorThreshold) return false;

  // Draw bounding rect around contour, determine what area should be of cross
  // with this width, compare to real area
  Rect rect = boundingRect(points);
  expected_area = 5 * pow(rect.width / 3.0, 2);
  error = fabs(contour_area / expected_area - 1);
  if (error > crossSideErrorThreshold) return false;

  // Find all angles in contour, then check that the mean angle is around 40
  // degrees and variance is low
  std::vector<double> angles;
  findAngles(points, angles);
  using namespace boost::accumulators;
  accumulator_set<int, features<tag::mean, tag::variance>> acc;
  for (double angle : angles) acc(angle);
  auto mean = boost::accumulators::mean(acc);
  auto variance = sqrt(boost::accumulators::variance(acc));
  // ~printf("CROSS SIDES=%d MEAN=%f VAR=%f\n",points.size(),mean,variance);
  if (fabs(mean - 40.0) > crossAngleMeanErrorThreshold) return false;
  if (variance > crossAngleVarErrorThreshold) return false;

  return true;
}

const double pi = 3.1415926;
bool GrayscaleContour::isCircle(std::vector<Point>& points) {
  if (points.size() < 5) return false;
  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);
  double expected_area, error;

  // Find a min enclosing circle for contour, then compare enclosing cirlcle
  // area to real area
  Point2f center;
  float radius;
  minEnclosingCircle(points, center, radius);
  expected_area = pi * pow(radius, 2);
  error = fabs(contour_area / expected_area - 1);
  if (error > circleEnclosingErrorThreshold) return false;

  // ~doub
  // ~Rect rect = boundingRect(points);
  // ~double contour_area = contourArea(points); //Actual area of the contour
  // ~double expected_area = pi*pow(rect.width/2.0,2); //What area should be if
  // contour is a circle
  // ~double error  = fabs(contour_area/expected_area-1);
  // ~if (error > 0.2) return false;
  // ~
  // ~double perimeter = arcLength(points, true);
  // ~double radius = perimeter/(2.0*pi);
  // ~expected_area = pi*pow(radius,2);
  // ~error = fabs(contour_area/expected_area-1.0);
  // ~if (error > 0.2) return false;

  // double center,radius;
  // minEnclosingCircle( points, center,radius);

  return true;
}
bool GrayscaleContour::filterArea(std::vector<Point> contour) {
  return contourArea(contour) < (minArea * (frame_width * frame_height));
}
Point GrayscaleContour::findCenter(std::vector<Point>& points) {
  double x = 0, y = 0;
  for (int i = 0; i < points.size(); i++) {
    x += points[i].x;
    y += points[i].y;
  }
  x /= points.size();
  y /= points.size();
  return Point(x, y);
}
double GrayscaleContour::findAngle(cv::Point& p1, cv::Point& p2,
                                   cv::Point& p3) {
  cv::Point v1 = p2 - p1;
  cv::Point v2 = p3 - p1;
  float m1 = sqrt(v1.x * v1.x + v1.y * v1.y);
  float m2 = sqrt(v2.x * v2.x + v2.y * v2.y);
  float thecos = v1.dot(v2) / (m1 * m2);

  return acos(thecos) * 180 / 3.1415;
}
void GrayscaleContour::findAngles(std::vector<Point>& points,
                                  std::vector<double>& angles) {
  for (int i = 0; i < points.size() - 2; i++)
    angles.push_back(findAngle(points[i], points[i + 1], points[i + 2]));
  angles.push_back(findAngle(points[points.size() - 2],
                             points[points.size() - 1], points[0]));
  angles.push_back(findAngle(points[points.size() - 1], points[0], points[1]));
}
void GrayscaleContour::setShapePoints(navigator_msgs::DockShape& dockShape,
                                      std::vector<Point>& points) {
  Point center = findCenter(points);
  dockShape.CenterX = center.x + roi.x;
  dockShape.CenterY = center.y + roi.y;
  dockShape.img_width = colorFrame.cols;
  for (int j = 0; j < points.size(); j++) {
    geometry_msgs::Point p;
    p.x = points[j].x + roi.x;
    p.y = points[j].y + roi.y;
    p.z = 0;
    dockShape.points.push_back(p);
  }
}
