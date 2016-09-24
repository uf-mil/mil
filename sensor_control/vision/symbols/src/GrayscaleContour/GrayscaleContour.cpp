#include "GrayscaleContour.h"
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

  #ifdef DO_DEBUG
  namedWindow("Menu",CV_WINDOW_AUTOSIZE);
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
  #endif
}
void GrayscaleContour::GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols)
{
  colorFrame = frame;
  CropFrame();
  ConvertToGrayscale();
  DetectEdges();
  FindContours();
  FindShapes();

  #ifdef DO_DEBUG
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
std::string GrayscaleContour::GetColor(std::vector<cv::Point>& shape)
{
  
}
void GrayscaleContour::FindShapes()
{
  shapes.clear();
  for (int i = 0; i < contours.size(); i++) {
    std::vector<cv::Point> approx;
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.025, true);
    shapes.push_back(approx);
  }
}
bool GrayscaleContour::filterArea(std::vector<Point> contour)
{
  return contourArea(contour) < minArea;
}
