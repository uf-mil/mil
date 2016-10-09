#pragma once
#include <navigator_msgs/DockShapes.h>
#include "../DockShapeVision.h"
#include "FrameProc.h"
#include "ShapeFind.h"
class ContourMethod : public DockShapeVision {
  ShapeFind blueFinder;
  ShapeFind redFinder;
  ShapeFind greenFinder;
  FrameProc fp;

 public:
  ContourMethod(ros::NodeHandle& nh);
  void GetShapes(cv::Mat& frame, cv::Rect roi,
                 navigator_msgs::DockShapes& symbols);
  void init();
};
