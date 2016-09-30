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
      double top;
      double bottom;
      double left;
      double right;
    };
    ROIparams roiParams;
    Rect roi;
    
    struct CannyParams
    {
      int thresh1;
      int thresh2;    
    };
    int epsilonFactor;
    CannyParams cannyParams;
    
    void CropFrame();
    void ConvertToGrayscale();
    void DetectEdges();
    void FindContours();
    void FindPolygons();
    bool GetColor(int shapeIndex,std::string& color);
    Point findCenter(std::vector<Point>& points);

    int frame_height;
    int frame_width;

    int CROSS_BOUNDING_AREA_LOW;
    int CROSS_BOUNDING_AREA_HIGH;
    int TRI_BOUNDING_AREA_LOW;
    int TRI_BOUNDING_AREA_HIGH;
    int CIRCLE_BOUNDING_AREA_LOW;
    int CIRCLE_BOUNDING_AREA_HIGH;
    
    bool filterArea(std::vector<Point> contour);
    static double minArea;
    
    static double contourAreaToBoundingRectAreaRatio(std::vector<cv::Point> &points);
    static double contourAreaToPerimeterRatio(std::vector<cv::Point> &points);
    static double sideLengthVariance(std::vector<cv::Point> &points);
    bool isTriangle(std::vector<Point>& points);
    bool isCross(std::vector<Point>& points);
    bool isCircle(std::vector<Point>& points);
  public:
    GrayscaleContour(ros::NodeHandle& nh);
    void GetShapes(cv::Mat &frame,cv::Rect roi,navigator_msgs::DockShapes& symbols);
    void init();
};
