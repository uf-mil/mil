#include <string>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>  // dont forget this include for camera stream functionality

using namespace std;

int main(int argc, char **argv)
{
  // Node setup
  string cam_topic {"stereo/left/image_rect_color"};
  size_t history_length{200};
  ros::init(argc, argv, "camera_stream_demo");
  ros::NodeHandle nh;

  // Template argument should be cv::Vec3b for color images or uint8_t 
  // For grayscale images
  nav::ROSCameraStream<cv::Vec3b> left_cam_stream(nh, history_length);  // Constructs empty inactive
                                                                        // camera stream object
  if(!left_cam_stream.init(cam_topic)) // Initializes object, if sucessful, object will automatically 
    return -1;                         // store a history of images published to cam_topic in its buffer

  // Display most recent and oldest frame in the buffer
  while(cv::waitKey(100) == -1 && left_cam_stream.ok())
  {
    size_t size = left_cam_stream.size();
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
