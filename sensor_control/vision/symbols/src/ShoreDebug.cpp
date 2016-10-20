#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <navigator_msgs/DockShape.h>
#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/GetDockShape.h>
#include <navigator_msgs/GetDockShapes.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include "opencv2/opencv.hpp"

//#include "PoseEstimator.h"

using namespace cv;
class ShoreDebug {
 private:
  bool hsv_windows;
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber foundShapesSubscriber;
  std::string camera_topic;
  std::string req_shape, req_color;
  Mat image, filtered_image;
  navigator_msgs::DockShapes shapes;
  struct ColorThresh {
    Scalar low;
    Scalar high;
  };
  ColorThresh red;
  ColorThresh red2;
  ColorThresh blue;
  ColorThresh green;

  // PoseEstimator poseEstimator;

  void drawShape(Mat &frame, navigator_msgs::DockShape &shape) {
    cv::circle(frame, Point(shape.CenterX, shape.CenterY), 4,
               Scalar(255, 255, 255), 5);
    putText(frame, shape.Shape + "(" + shape.Color + ")",
            Point(shape.CenterX - 25, shape.CenterY - 25), 4, 1,
            Scalar(0, 0, 0), 3);
  }
  void newFrameCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image = cv_ptr->image;
    if (hsv_windows) {
      Mat hsv_frame, redFrame, blueFrame, greenFrame, rtemp, rtemp2;
      cvtColor(image, hsv_frame, CV_BGR2HSV);
      inRange(hsv_frame, red.low, red.high, rtemp);
      inRange(hsv_frame, red2.low, red2.high, rtemp2);
      inRange(hsv_frame, blue.low, blue.high, blueFrame);
      inRange(hsv_frame, green.low, green.high, greenFrame);
      redFrame = rtemp | rtemp2;
      imshow("blue", blueFrame);
      imshow("green", greenFrame);
      imshow("red", redFrame);
    }
    for (navigator_msgs::DockShape symbol : shapes.list) {
      drawShape(image, symbol);
      // poseEstimator.process(image, symbol);
    }
    imshow("Result", image);
  }
  void foundShapesCallback(const navigator_msgs::DockShapes &ds) {
    shapes = ds;
  }
  void initParams() {
    // Set HSV values
    nh.getParam("hsv/red1/low/H", red.low[0]);
    nh.getParam("hsv/red1/low/S", red.low[1]);
    nh.getParam("hsv/red1/low/V", red.low[2]);
    nh.getParam("hsv/red1/high/H", red.high[0]);
    nh.getParam("hsv/red1/high/S", red.high[1]);
    nh.getParam("hsv/red1/high/V", red.high[2]);

    nh.getParam("hsv/red2/low/H", red2.low[0]);
    nh.getParam("hsv/red2/low/S", red2.low[1]);
    nh.getParam("hsv/red2/low/V", red2.low[2]);
    nh.getParam("hsv/red2/high/H", red2.high[0]);
    nh.getParam("hsv/red2/high/S", red2.high[1]);
    nh.getParam("hsv/red2/high/V", red2.high[2]);

    nh.getParam("hsv/blue/low/H", blue.low[0]);
    std::cout << "hsv/blue/low/H " << blue.low[0] << std::endl;
    nh.getParam("hsv/blue/low/S", blue.low[1]);
    nh.getParam("hsv/blue/low/V", blue.low[2]);
    nh.getParam("hsv/blue/high/H", blue.high[0]);
    nh.getParam("hsv/blue/high/S", blue.high[1]);
    nh.getParam("hsv/blue/high/V", blue.high[2]);

    nh.getParam("hsv/green/low/H", green.low[0]);
    nh.getParam("hsv/green/low/S", green.low[1]);
    nh.getParam("hsv/green/low/V", green.low[2]);
    nh.getParam("hsv/green/high/H", green.high[0]);
    nh.getParam("hsv/green/high/S", green.high[1]);
    nh.getParam("hsv/green/high/V", green.high[2]);
    nh.getParam("hsv_windows", hsv_windows);
    nh.param<std::string>("symbol_camera", camera_topic,
                          "/right_camera/image_color");

    std::cout << "Using Camera: " << camera_topic << std::endl;
    if (hsv_windows)
      std::cout << "Displaying HSV Threshold Windows" << std::endl;
  }

 public:
  ShoreDebug() : nh("dock_shape_shore_debug"), it_(nh) {
    red = ColorThresh{Scalar(0, 10, 100), Scalar(30, 255, 255)};
    red2 = ColorThresh{Scalar(155, 10, 100), Scalar(180, 255, 255)};
    blue = ColorThresh{Scalar(90, 100, 100), Scalar(150, 255, 255)};
    green = ColorThresh{Scalar(30, 85, 25), Scalar(85, 255, 255)};
    initParams();
    image_sub_ =
        it_.subscribe(camera_topic, 1, &ShoreDebug::newFrameCallback, this);
    foundShapesSubscriber =
        nh.subscribe("/dock_shapes/found_shapes", 1000,
                     &ShoreDebug::foundShapesCallback, this);
    namedWindow("Result", CV_WINDOW_AUTOSIZE);
    namedWindow("GetShapes", CV_WINDOW_AUTOSIZE);
    if (hsv_windows) {
      namedWindow("blue", CV_WINDOW_AUTOSIZE);
      namedWindow("red", CV_WINDOW_AUTOSIZE);
      namedWindow("green", CV_WINDOW_AUTOSIZE);
    }
  }

  void getShapes() {
    navigator_msgs::GetDockShapes x;
    if (ros::service::call("/dock_shapes/GetShapes", x)) {
      filtered_image = image;
      for (auto shape = x.response.shapes.list.begin();
           shape != x.response.shapes.list.end(); shape++)
        drawShape(filtered_image, *shape);
      imshow("GetShapes", filtered_image);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_shore_debug");
  ShoreDebug sd;
  while (ros::ok()) {
    int x = waitKey(50);
    if (x != -1) {
      if (x == 27)
        break;
      else if (x == 32)
        sd.getShapes();
      // std::cout << "Key" << x << std::endl;
    }
    ros::spinOnce();
  }
  return 0;
}
