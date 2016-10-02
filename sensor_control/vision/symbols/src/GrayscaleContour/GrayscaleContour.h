#pragma once
#include "../DockShapeVision.h"
#include <navigator_msgs/DockShapes.h>
#include "opencv2/opencv.hpp"
#include <memory>
using namespace cv;

//#define DO_DEBUG
#define DO_ROS_DEBUG
#ifdef DO_ROS_DEBUG
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#endif
class GrayscaleContour : public DockShapeVision
{
  private:
    Mat colorFrame,croppedFrame,grayscaleFrame,edgesFrame;
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > shapes;
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
    Mat contoursFrame;
    int frame_height;
    int frame_width;
    
    bool filterArea(std::vector<Point> contour);
    static double minArea;
    
    bool isTriangle(std::vector<Point>& points);
    bool isCross(std::vector<Point>& points);
    bool isCircle(std::vector<Point>& points);

    #ifdef DO_ROS_DEBUG
    std::unique_ptr<image_transport::ImageTransport> image_transport;
    image_transport::Publisher color_debug_publisher;
    image_transport::Publisher contour_debug_publisher;
    #endif
  public:
    GrayscaleContour(ros::NodeHandle& nh);
    void GetShapes(cv::Mat &frame,cv::Rect roi,navigator_msgs::DockShapes& symbols);
    void init();
};
