#include "GrayscaleContour.h"
#include <sstream>
int GrayscaleContour::minArea = 100;
GrayscaleContour::GrayscaleContour(ros::NodeHandle& nh) :
  DockShapeVision(nh)
{
  Mat colorFrame,croppedFrame,grayscaleFrame,edgesFrame;
  
  roiParams.top = 103;
  roiParams.bottom = 346;
  roiParams.left = 73;
  roiParams.right = 572;
  cannyParams.thresh1 = 75;
  cannyParams.thresh2 = 100;

  epsilonFactor = 40;

  CROSS_BOUNDING_AREA_LOW = 430;
  CROSS_BOUNDING_AREA_HIGH = 470;
  TRI_BOUNDING_AREA_LOW = 410;
  TRI_BOUNDING_AREA_HIGH = 500;
  CIRCLE_BOUNDING_AREA_LOW = 550;
  CIRCLE_BOUNDING_AREA_HIGH = 650;
  #ifdef DO_DEBUG
  namedWindow("Result",CV_WINDOW_AUTOSIZE);
  namedWindow("Menu",CV_WINDOW_AUTOSIZE);
  namedWindow("ShapeParams",CV_WINDOW_AUTOSIZE);
  namedWindow("Result",CV_WINDOW_AUTOSIZE);
  namedWindow("Color Cropped",CV_WINDOW_AUTOSIZE);
  namedWindow("Grayscale",CV_WINDOW_AUTOSIZE);
  namedWindow("Contours",CV_WINDOW_AUTOSIZE);
  namedWindow("Edges",CV_WINDOW_AUTOSIZE);
  #endif
}
void GrayscaleContour::init()
{
  #ifdef DO_DEBUG
  createTrackbar("thresh1", "Menu", &cannyParams.thresh1, 500);
  createTrackbar("thresh2", "Menu", &cannyParams.thresh2, 255);
  createTrackbar("top", "Menu", &roiParams.top, 482);
  createTrackbar("bottom", "Menu", &roiParams.bottom,  482);
  createTrackbar("left", "Menu", &roiParams.left, 644);
  createTrackbar("right", "Menu", &roiParams.right, 644);
  createTrackbar("minArea", "Menu", &minArea, 2000);
  createTrackbar("Epsilon  factor (x1000)", "Menu", &epsilonFactor, 2000);
  
  createTrackbar("CROSS_BOUNDING_AREA_LOW ", "ShapeParams", &CROSS_BOUNDING_AREA_LOW , 1000);
  createTrackbar("CROSS_BOUNDING_AREA_HIGH ", "ShapeParams", &CROSS_BOUNDING_AREA_HIGH , 1000);
  createTrackbar("TRI_BOUNDING_AREA_LOW ", "ShapeParams", &TRI_BOUNDING_AREA_LOW , 1000);
  createTrackbar("TRI_BOUNDING_AREA_HIGH ", "ShapeParams", &TRI_BOUNDING_AREA_HIGH , 1000);
  createTrackbar("CIRCLE_BOUNDING_AREA_LOW", "ShapeParams", &CIRCLE_BOUNDING_AREA_LOW , 1000);
  createTrackbar("CIRCLE_BOUNDING_AREA_HIGH ", "ShapeParams", &CIRCLE_BOUNDING_AREA_HIGH , 1000);
  #endif
}
void GrayscaleContour::GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols)
{
  colorFrame = frame;
  CropFrame();
  ConvertToGrayscale();
  DetectEdges();
  FindContours();
  FindPolygons();

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
    
    Point center = findCenter(shape);
    dockShape.CenterX = center.x;
    dockShape.CenterY = center.y;
    
    dockShape.img_width = colorFrame.cols;

    //TransformPointsToUncropped(shape);
    for (int j = 0; j < shape.size(); j++) {
      geometry_msgs::Point p;
      p.x = shape[j].x;
      p.y = shape[j].y;
      p.z = 0;
      dockShape.points.push_back(p);
    }
    symbols.list.push_back(dockShape); 
  }
  #ifdef DO_DEBUG
  Mat result = croppedFrame.clone();
  for (auto symbol : symbols.list) {
    cv::circle(result,Point(symbol.CenterX,symbol.CenterY),4,Scalar(255,255,255),5);
    putText(result, symbol.Shape + "\n(" + symbol.Color + ")", Point(symbol.CenterX-100, symbol.CenterY-25),1 , 1, Scalar(0,0,0),  3);
  }
  imshow("Result",result);
  imshow("Color Cropped",croppedFrame);
  imshow("Grayscale",grayscaleFrame);
  imshow("Edges",edgesFrame);
  #endif
}
void GrayscaleContour::CropFrame()
{
  cv::Rect roi(roiParams.left,roiParams.top,WIDTH-roiParams.left-(WIDTH-roiParams.right),HEIGHT-roiParams.top-(HEIGHT-roiParams.bottom));
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
  
  #ifdef DO_DEBUG
  Mat contoursFrame = Mat(edgesFrame.size(), edgesFrame.type(),Scalar(0,0,0));
  for (int i = 0; i < contours.size(); i++)
  {
    //Scalar color( rand()&255, rand()&255, rand()&255 );
    drawContours(contoursFrame, contours, i, Scalar(255,255,255));
  }
  imshow("Contours",contoursFrame);
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
    //For now just if it has 3 sides
  return points.size() == 3;
  //cv::Rect boundingRect = cv::boundingRect(points);
  //float area = contourArea(points) / (boundingRect.width * boundingRect.height);
  //if (area >=  (float(TRI_BOUNDING_AREA_LOW)/1000) && area <= (float(TRI_BOUNDING_AREA_LOW)/1000) ) return true;
  //else return false;
}
bool GrayscaleContour::isCross(std::vector<Point>& points)
{
  //For now just if it has 12 sides
  return points.size() == 12;
  //cv::Rect boundingRect = cv::boundingRect(points);
  //float area = contourArea(points) / (boundingRect.width * boundingRect.height);
  //if (area >= (float(CROSS_BOUNDING_AREA_LOW)/1000) && area <= (float(CROSS_BOUNDING_AREA_HIGH)/1000)) return true;
  //else return false;
}
bool GrayscaleContour::isCircle(std::vector<Point>& points)
{
  if (points.size() < 5) return false;
  RotatedRect ellipse = fitEllipse(points);
  float area = contourArea(points) / (ellipse.size.height * ellipse.size.width);
  if (area >= (float(CIRCLE_BOUNDING_AREA_LOW)/1000) && area <= (float(CIRCLE_BOUNDING_AREA_HIGH)/1000)) return true;
  return false;
}
bool GrayscaleContour::filterArea(std::vector<Point> contour)
{
  return contourArea(contour) < minArea;
}
double GrayscaleContour::contourAreaToBoundingRectAreaRatio(std::vector<cv::Point> &points)
{
  cv::Rect boundingRect = cv::boundingRect(points);
  double rect_area = boundingRect.width * boundingRect.height;
  double contour_area = contourArea(points);
  return contour_area / rect_area;
}
double GrayscaleContour::contourAreaToPerimeterRatio(std::vector<cv::Point> &points)
{
  double area = contourArea(points);
  double perimeter = arcLength(points, true);
  return area / perimeter;
}
double GrayscaleContour::sideLengthVariance(std::vector<cv::Point> &points)
{
  
}
void GrayscaleContour::TransformPointsToUncropped(std::vector<Point>& points)
{
 for (auto p : points)
 {
   p.x += roiParams.left;
   p.y += roiParams.top;
 } 
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
