#include <string>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_acquisition/ros_camera_stream.hpp>  // dont forget this include for camera stream functionality

using namespace std;

int main(int argc, char **argv)
{
  // Node setup
  string cam_topic {"stereo/left/image_rect_color"};
  size_t history_length{200};
  ros::init(argc, argv, "camera_stream_demo");
  ros::NodeHandle nh;

  // These two lines are all you need to create a self updating buffer
  // of images published to an image topic. 
  // Template argument should be cv::Vec3b for color images or uint8_t 
  // For grayscale images
  nav::ROSCameraStream<cv::Vec3b> left_cam_stream(nh, history_length);
  left_cam_stream.init(cam_topic);

  // Display most recent and oldest frame in the buffer
  while(cv::waitKey(100) == -1)
  {
    size_t size = left_cam_stream.length();
    if(size == 200)
    {
      cv::imshow("newest_frame", left_cam_stream[0]->image());
      cv::imshow("oldest_frame", left_cam_stream[-1]->image());

      // It is also possible to request a frame taken at a specific time
      /*
      Mat frame_at_time_t = left_cam_stream.getFrameFromTime(desired_time)->image();
      */
    }
  }

}
