#include "GrayscaleContour.h"
#include <sstream>
double GrayscaleContour::minArea = 100.0/(644.0*482.0);
GrayscaleContour::GrayscaleContour(ros::NodeHandle& nh) :
  DockShapeVision(nh)
{
  //Set ros params
  nh.param<int>("grayscale/canny/thresh1", cannyParams.thresh1,60);
  nh.param<int>("grayscale/canny/thresh2", cannyParams.thresh2,100);

  epsilonFactor =  3;

  #ifdef DO_DEBUG
  namedWindow("Menu",CV_WINDOW_AUTOSIZE);
  namedWindow("Result",CV_WINDOW_AUTOSIZE);
  namedWindow("Grayscale",CV_WINDOW_AUTOSIZE);
  namedWindow("Contours",CV_WINDOW_AUTOSIZE);
  namedWindow("Edges",CV_WINDOW_AUTOSIZE);
  #endif
  #ifdef DO_ROS_DEBUG
  image_transport.reset(new image_transport::ImageTransport(nh));
  color_debug_publisher = image_transport->advertise("/dock_shapes/finder/debug_color", 1);
  contour_debug_publisher = image_transport->advertise("/dock_shapes/finder/debug_contours", 1);
  #endif
}
void GrayscaleContour::init()
{
  #ifdef DO_DEBUG
  createTrackbar("thresh1", "Menu", &cannyParams.thresh1, 500);
  createTrackbar("thresh2", "Menu", &cannyParams.thresh2, 255);
  createTrackbar("Epsilon  factor (x1000)", "Menu", &epsilonFactor, 2000);
  #endif
}
void GrayscaleContour::GetShapes(cv::Mat &frame,cv::Rect roi,navigator_msgs::DockShapes& symbols)
{
  if (frame.empty()) return;

  this->roi = roi;
  frame_width = frame.cols;
  frame_height = frame.rows;
  
  colorFrame = frame;
  CropFrame();
  ConvertToGrayscale();
  DetectEdges();
  FindContours();
  FindPolygons();

  #ifdef DO_DEBUG
  Mat result = croppedFrame.clone();
  #endif
  #ifdef DO_ROS_DEBUG
  cv_bridge::CvImage ros_color_debug;
  ros_color_debug.encoding = "bgr8";
  ros_color_debug.image = croppedFrame.clone();
  #endif
  
  for (int i = 0; i < shapes.size(); i++)
  {
    auto shape = shapes.at(i);
    navigator_msgs::DockShape dockShape;
    
    if (isTriangle(shape))
    {
      dockShape.Shape = navigator_msgs::DockShape::TRIANGLE;
    } else if (isCross(shape))
    {
      dockShape.Shape = navigator_msgs::DockShape::CROSS;
    } else if (isCircle(shape))
    {
      dockShape.Shape = navigator_msgs::DockShape::CIRCLE;
    } else continue;

    if (!GetColor(i,dockShape.Color)) continue;
    
    #ifdef DO_DEBUG
    drawContours(result, shapes, i, Scalar(0,0,255) );
    #endif
    #ifdef DO_ROS_DEBUG
    drawContours(ros_color_debug.image, shapes, i, Scalar(0,0,255) );
    #endif
    
    Point center = findCenter(shape);
    dockShape.CenterX = center.x + roi.x;
    dockShape.CenterY = center.y + roi.y;
    dockShape.img_width = colorFrame.cols;
    for (int j = 0; j < shape.size(); j++) {
      geometry_msgs::Point p;
      p.x = shape[j].x + roi.x;
      p.y = shape[j].y + roi.y;
      p.z = 0;
      dockShape.points.push_back(p);
    }
    symbols.list.push_back(dockShape); 
  }

  #ifdef DO_DEBUG
  for (auto symbol : symbols.list) {
    putText(result, symbol.Shape + "\n(" + symbol.Color + ")", Point(symbol.CenterX+10-roi.x, symbol.CenterY-roi.y),1 , 1, Scalar(0,0,0),  3);
  }
  imshow("Result",result);
  imshow("Grayscale",grayscaleFrame);
  imshow("Edges",edgesFrame);
  imshow("Contours",contoursFrame);
  #endif
  #ifdef DO_ROS_DEBUG
  for (auto symbol : symbols.list) {
    putText(ros_color_debug.image, symbol.Shape + "\n(" + symbol.Color + ")", Point(symbol.CenterX+10-roi.x, symbol.CenterY-roi.y),1 , 1, Scalar(0,0,0),  3);
  }
  color_debug_publisher.publish(ros_color_debug.toImageMsg());
  cv_bridge::CvImage ros_contours_debug;
  ros_color_debug.encoding = "mono8";
  ros_color_debug.image = contoursFrame.clone();
  contour_debug_publisher.publish(ros_color_debug.toImageMsg());
  #endif
}
void GrayscaleContour::CropFrame()
{
  croppedFrame = colorFrame(roi);
}
void GrayscaleContour::ConvertToGrayscale()
{
  cvtColor(croppedFrame, grayscaleFrame, CV_BGR2GRAY );
}
void GrayscaleContour::DetectEdges()
{
  Canny(grayscaleFrame,edgesFrame,cannyParams.thresh1,cannyParams.thresh2);
}
void GrayscaleContour::FindContours()
{
  contours.clear();
  hierarchy.clear();
  findContours(edgesFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  //Filter out very small contours
  auto cit = contours.begin();
  auto hit = hierarchy.begin(); 
  while (cit != contours.end())
  {
    if (filterArea(*cit))
    {
      cit = contours.erase(cit);
      hit = hierarchy.erase(hit);
    }
    else {
      cit++;
      hit++;
    }
  }

  #if defined(DO_DEBUG) || defined(DO_ROS_DEBUG)
  contoursFrame = Mat(edgesFrame.size(), edgesFrame.type(),Scalar(0,0,0));
  for (int i = 0; i < contours.size(); i++)
  {
    drawContours(contoursFrame, contours, i, Scalar(255,255,255));
  }
  #endif
}
bool GrayscaleContour::GetColor(int shapeIndex,std::string& color)
{
  Mat mask = Mat::zeros(croppedFrame.rows, croppedFrame.cols, CV_8UC1);
  drawContours(mask,shapes,shapeIndex, Scalar(255), CV_FILLED);
  Scalar meanColor = mean(croppedFrame,mask);
  //std::cout << meanColor << std::endl;
  std::ostringstream unknown;
  unknown << "UKNOWN: " << meanColor.val[0] << " " << meanColor.val[1] << " " << meanColor.val[2] << std::endl;
  double sum = meanColor.val[0] + meanColor.val[1] + meanColor.val[2];
  if (sum > 500) return false;
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
void GrayscaleContour::FindPolygons()
{
  shapes.clear();
  for (int i = 0; i < contours.size(); i++) {
    std::vector<cv::Point> approx;
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * (float(epsilonFactor)/1000) , true);
    shapes.push_back(approx);
  }
}
bool GrayscaleContour::isTriangle(std::vector<Point>& points)
{
  if (points.size() < 3) return false;

  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);
  double expected_area,expected_perimeter,error;
  
  cv::Rect boundingRect = cv::boundingRect(points);
  expected_area = 0.5 * boundingRect.width * boundingRect.height;
  error = fabs(contour_area/expected_area-1.0);
  if (error > 0.1) return false;
  
  expected_perimeter = boundingRect.width*3;
  error = fabs(perimeter/expected_perimeter-1.0);
  if (error > 0.1) return false;
  //float area = contourArea(points) / (boundingRect.width * boundingRect.height);
  //if (area >=  (float(TRI_BOUNDING_AREA_LOW)/1000) && area <= (float(TRI_BOUNDING_AREA_LOW)/1000) ) return true;
  //else return false;
  return true;
}
bool GrayscaleContour::isCross(std::vector<Point>& points)
{
  //Check area/size length
  if (points.size() < 10) return false;

  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);  
  double expected_area,error;

  double side_length = perimeter / 12.0;
  expected_area = 5.0*pow(side_length,2);
  error = fabs(contour_area/expected_area-1.0);
  if (error > 0.1) return false;

  Rect rect = boundingRect(points);
  expected_area = 5*pow( rect.width/3.0,2);
  error  = fabs(contour_area/expected_area-1);
  if (error > 0.1) return false;
  
  return true;
}

const double pi = 3.1415926;
bool GrayscaleContour::isCircle(std::vector<Point>& points)
{
  if (points.size() < 5) return false;

  double contour_area = contourArea(points);
  double perimeter = arcLength(points, true);  
  double expected_area,error;
  //bounding rect area test
  Point2f center;
  float radius;
  minEnclosingCircle(points,center,radius);
  expected_area = pi*pow(radius,2);
  error  = fabs(contour_area/expected_area-1);
  if (error > 0.2) return false; 
  
  // ~doub
  // ~Rect rect = boundingRect(points);
  // ~double contour_area = contourArea(points); //Actual area of the contour
  // ~double expected_area = pi*pow(rect.width/2.0,2); //What area should be if contour is a circle
  // ~double error  = fabs(contour_area/expected_area-1);
  // ~if (error > 0.2) return false;
// ~
  // ~double perimeter = arcLength(points, true);
  // ~double radius = perimeter/(2.0*pi);
  // ~expected_area = pi*pow(radius,2);
  // ~error = fabs(contour_area/expected_area-1.0);
  // ~if (error > 0.2) return false;
  
  //double center,radius;
  //minEnclosingCircle( points, center,radius);

  return true;
}
bool GrayscaleContour::filterArea(std::vector<Point> contour)
{
  return contourArea(contour) < (minArea * (frame_width*frame_height) ) ;
}
Point  GrayscaleContour::findCenter(std::vector<Point>& points) {
  int x = 0, y = 0;
  for (int i = 0; i < points.size(); i++) {
    x += points[i].x;
    y += points[i].y;
  }
  x /= points.size();
  y /= points.size();
  return Point(x, y);
}
