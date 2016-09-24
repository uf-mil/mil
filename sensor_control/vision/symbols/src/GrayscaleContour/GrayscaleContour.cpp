#include "GrayscaleContour.h"
GrayscaleContour::GrayscaleContour(ros::NodeHandle& nh) :
  DockShapeVision(nh)
{
  Mat colorFrame,croppedFrame,grayscaleFrame,edgesFrame;
  
  roiParams.top = 103;
  roiParams.bottom = 346;
  roiParams.left = 73;
  roiParams.right = 572;

  cannyParams.thresh1 = 255;
  cannyParams.thresh2 = 100;

  #ifdef DO_DEBUG
  namedWindow("Menu",CV_WINDOW_AUTOSIZE);
  namedWindow("Color Cropped",CV_WINDOW_AUTOSIZE);
  namedWindow("Grayscale",CV_WINDOW_AUTOSIZE);
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
