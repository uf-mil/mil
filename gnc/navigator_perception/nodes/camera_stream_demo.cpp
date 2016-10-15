#include <string>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros_camera_stream.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

using namespace std;
using namespace nav;


int main(int argc, char **argv)
{
  std::string cam_topic {"stereo/left/image_rect_color"};
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  nav::ROSCameraStream<cv::Vec3b, float> left_cam_stream(nh, 200);
  left_cam_stream.init("stereo/left/image_rect_color");


  while(cv::waitKey(100) == -1)
  {
    size_t size = left_cam_stream.length();
    if(size == 200)
    {
      cv::imshow("newest_frame", left_cam_stream[0]->image());
      cv::imshow("oldest_frame", left_cam_stream[-1]->image());
    }
  }

}
