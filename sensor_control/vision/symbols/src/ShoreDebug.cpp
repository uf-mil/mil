#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <navigator_msgs/GetDockShape.h>
#include <navigator_msgs/GetDockShapes.h>
#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/DockShape.h>

#include "PoseEstimator.h"

using namespace cv;
class ShoreDebug {
 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber foundShapesSubscriber;
  std::string camera_topic;
  std::string req_shape,req_color;
  Mat image,filtered_image;
  navigator_msgs::DockShapes shapes;
  
  PoseEstimator poseEstimator;
  
  void drawShape(Mat& frame, navigator_msgs::DockShape& shape)
  {
    cv::circle(frame,Point(shape.CenterX,shape.CenterY),4,Scalar(255,255,255),5);
    putText(frame, shape.Shape + "(" + shape.Color + ")", Point(shape.CenterX-25, shape.CenterY-25), 4, 1, Scalar(0,0,0),  3); 
  }
  void newFrameCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image = cv_ptr->image;
    for (navigator_msgs::DockShape symbol : shapes.list) {
        drawShape(image,symbol);
        poseEstimator.process(image, symbol);
    }
    imshow("Result",image);    
  }
  void foundShapesCallback(const navigator_msgs::DockShapes &ds)
  {
    shapes = ds;
  }
 public:
  ShoreDebug() : 
    nh_("dock_shape_shore_debug"),
    it_(nh_)
  {
    nh_.param<std::string>("symbol_camera", camera_topic, "/right_camera/image_color");
    nh_.param<std::string>("color", req_color, navigator_msgs::DockShape::BLUE);
    nh_.param<std::string>("shape", req_shape, navigator_msgs::DockShape::CROSS);
    std::cout << "Using Camera: " << camera_topic << std::endl;
    image_sub_ = it_.subscribe(camera_topic, 1, &ShoreDebug::newFrameCallback, this);
    foundShapesSubscriber = nh_.subscribe("/dock_shapes/found_shapes", 1000, &ShoreDebug::foundShapesCallback, this);
    namedWindow("Result",CV_WINDOW_AUTOSIZE);
    namedWindow("GetShape",CV_WINDOW_AUTOSIZE);
  }

  void getShapes()
  {
    navigator_msgs::GetDockShapes x;
    if (ros::service::call("/dock_shapes/GetShapes", x))
    {
      filtered_image = image;
      for (auto shape = x.response.shapes.list.begin(); shape !=  x.response.shapes.list.end(); shape++)
        drawShape(filtered_image,*shape);
      imshow("GetShapes",filtered_image);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_shore_debug");
  ShoreDebug sd;
  while (ros::ok()) {
    int x = waitKey(50);
    if (x != -1)
    {
      if (x == 27) break;
      else if (x == 32) sd.getShapes();
      //std::cout << "Key" << x << std::endl;
    }
    ros::spinOnce();
  }
  return 0;
}
