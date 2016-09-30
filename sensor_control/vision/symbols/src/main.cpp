#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "DebugWindow.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include <navigator_msgs/DockShapes.h>

#include "std_srvs/SetBool.h"
#include "ContourMethod/ContourMethod.h"
#include "GrayscaleContour/GrayscaleContour.h"
//#include "FeatureDetectorMethod/FeatureDetectorMethod.h"
#include "DockShapeVision.h"
#include "sensor_msgs/RegionOfInterest.h"
#include <navigator_msgs/SetROI.h>

using namespace cv;

class ShooterVision {
  private:
    std::unique_ptr<DockShapeVision> vision;
    // ros frame thing
    navigator_msgs::DockShapes symbols;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher foundShapesPublisher;

    ros::ServiceServer runService;
    ros::ServiceServer roiService;
    std::string camera_topic;
    bool active;
    cv::Rect roi;
    int width;
    int height;
  public:
    ShooterVision() :
      nh_("dock_shape_finder"),
      it_(nh_)
    {
      vision.reset(new GrayscaleContour(nh_));
      vision->init();
      nh_.param<std::string>("symbol_camera", camera_topic, "/right/right/image_raw");
      runService = nh_.advertiseService("run", &ShooterVision::runCallback, this);
      roiService = nh_.advertiseService("setROI", &ShooterVision::roiServiceCallback, this);
      //#ifdef DO_DEBUG
      //DebugWindow::init();
      //#endif
      foundShapesPublisher = nh_.advertise<navigator_msgs::DockShapes>("/dock_shapes/found_shapes", 1000);
      image_sub_ = it_.subscribe(camera_topic, 1, &ShooterVision::run, this);

      roi = Rect(73,103,499,243);
    }

    bool runCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
      active = req.data;
      res.success = true;
      return true;
    }
    bool roiServiceCallback(navigator_msgs::SetROI::Request &req, navigator_msgs::SetROI::Response &res) {
      if (req.roi.x_offset < 0 ||
          req.roi.y_offset < 0 ||
          req.roi.x_offset + req.roi.width > width ||
          req.roi.y_offset + req.roi.height > height) {
        res.error = "OUTSIDE_OF_FRAME";
        res.success = false;
        return true;
      }
      roi = Rect(req.roi.x_offset,req.roi.y_offset,req.roi.width,req.roi.height);
      res.success = true;
      return true;
    }
    void run(const sensor_msgs::ImageConstPtr &msg) {
      if (!active) return;
      // Grab ros frame
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::Mat frame = cv_ptr->image;
      width = frame.cols;
      height = frame.rows;
      cv::waitKey(3);
      symbols.list.clear();

      vision->GetShapes(frame,roi,symbols);
      // Publish to ros
      //#ifdef DO_DEBUG
      //DebugWindow::UpdateResults(symbols);
      //#endif

      foundShapesPublisher.publish(symbols);
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_finder");
  ShooterVision sv = ShooterVision();
  while (waitKey(50) == -1 && ros::ok()) {
    ros::spin();
  }
  std::cout << "Key detected, exiting" << std::endl;
  return 0;
}
