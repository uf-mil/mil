#include "std_msgs/String.h"

#include "FrameProc.h"
#include "ShapeFind.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "DebugWindow.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "navigator_msgs/DockShapes.h"

#include "std_srvs/SetBool.h"

using namespace cv;

class ShooterVision {
  private:
    // ros frame thing
    navigator_msgs::DockShapes symbols;
    FrameProc fp;
    ShapeFind blueFinder;
    ShapeFind redFinder;
    ShapeFind greenFinder;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher chatter_pub;

    ros::ServiceServer serviceCommand;
    std::string camera_topic;
    bool active;

  public:
    ShooterVision() :
      nh_("dock_shape_processor"),
      it_(nh_), fp(), 
      blueFinder(navigator_msgs::DockShape::BLUE), 
      redFinder(navigator_msgs::DockShape::RED),
      greenFinder(navigator_msgs::DockShape::GREEN)
    {
      active = false;
      fp.init(&nh_);
      nh_.param<std::string>("symbol_camera", camera_topic, "/right_camera/image_color");
      serviceCommand = nh_.advertiseService("/dock_shapes/runvision", &ShooterVision::getShapeController, this);
      #ifdef DO_DEBUG
      DebugWindow::init();
      #endif

      chatter_pub = nh_.advertise<navigator_msgs::DockShapes>("/dock_shapes/found_shapes", 1000);
      image_sub_ = it_.subscribe(camera_topic, 1, &ShooterVision::run, this);
    }

    bool getShapeController(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
      active = req.data;
      std::cout<<"Setting active to "<<active<<std::endl;
      return true;
    }
    void run(const sensor_msgs::ImageConstPtr &msg) {
      // Grab ros frame
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      // Convert Ros frame to opencv
      cv::waitKey(3);
      if(active) {
        // Process frame
        fp.Prepare(cv_ptr->image);
        symbols.list.clear();

        // Find shapes in each color
        blueFinder.GetSymbols(fp.GetBlue(), &symbols);
        redFinder.GetSymbols(fp.GetRed(), &symbols);
        greenFinder.GetSymbols(fp.GetGreen(), &symbols);

        // Publish to ros
        #ifdef DO_DEBUG
        DebugWindow::UpdateResults(symbols);
        #endif

        chatter_pub.publish(symbols);
      }
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dock_shape_processor");
  ShooterVision sv = ShooterVision();
  while (waitKey(50) == -1 && ros::ok()) {
    ros::spin();
  }
  std::cout << "Key detected, exiting" << std::endl;
  return 0;
}
