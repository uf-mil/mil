#pragma once
#include "../DockShapeVision.h"
#include <navigator_msgs/DockShapes.h>
#include "opencv2/opencv.hpp"
using namespace cv;

#define DO_DEBUG
class GrayscaleContour : public DockShapeVision
{
  private:
    Mat colorFrame,croppedFrame,grayscaleFrame,edgesFrame;
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > shapes;

    struct ROIparams
    {
      int top;
      int bottom;
      int left;
      int right;
    };
    ROIparams roiParams;

    struct CannyParams
    {
      int thresh1;
      int thresh2;    
    };
    CannyParams cannyParams;
    
    void CropFrame();
    void ConvertToGrayscale();
    void DetectEdges();
    void FindContours();
    void FindShapes();

    static const int WIDTH = 644;
    static const int HEIGHT = 482;
    
    static bool filterArea(std::vector<Point> contour);
    static int minArea;
  public:
    GrayscaleContour(ros::NodeHandle& nh);
    void GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols);
    void init();
};
