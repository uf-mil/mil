#include "DockShapeVision.h"
using namespace cv;
int DockShapeVision::fontFace = CV_FONT_HERSHEY_SIMPLEX;
double DockShapeVision::fontScale = .6;
DockShapeVision::DockShapeVision(ros::NodeHandle& nh) : nh(nh) {}
void DockShapeVision::DrawShapes(cv::Mat& frame, navigator_msgs::DockShapes& symbols)
{
  for (auto symbol : symbols.list) {
    Scalar color(0,0,255);
    if (symbol.Color == navigator_msgs::DockShape::RED)
      color = Scalar(0,255,0);
    putText(frame, symbol.Shape + " (" + symbol.Color + ")",
            Point(symbol.CenterX, symbol.CenterY), fontFace, fontScale,
            Scalar(0,0,255), 1,CV_AA);
  }
}
