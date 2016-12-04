#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <navigator_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <navigator_vision_lib/image_filtering.hpp>

#include <missions/underwater_shape_identification.hpp>

using namespace std;

int main(int argc, char** argv) 
{ 
  cout << "\033[1;31mUnderwater Shape Identification Vision\033[0m" << endl;  

  // set up image acquisition
  string cam_topic {"down/image_raw"};
  size_t history_length{200};
  string challenge_name = "underwater_shape_identification";
  ros::init(argc, argv, challenge_name + "_vision");
  ros::NodeHandle nh;

  int img_buffer_size = 0;
  string name_space{challenge_name + "/"};
  nh.param<int>(name_space + "buffer_size", img_buffer_size, 5);
  nav::UnderwaterShapeDetector underwater_shape_detector(nh, img_buffer_size, name_space);
  // cv::Vec3b to get color imgs
  nav::ROSCameraStream<cv::Vec3b> left_cam_stream(nh, history_length);
  if(!left_cam_stream.init(cam_topic))
    return -1;

  // Display most recent frame in the buffer
  while(left_cam_stream.ok())
  {
    size_t size = left_cam_stream.size();
    if(size > 0)
    {
      cv::Mat latest = left_cam_stream[0]->image();
      cv::Mat _latest = latest;
      // cv::cvtColor(latest, _latest, CV_BGR2RGB);
      cv::imshow("newest_frame", _latest);
    }
    char code = cv::waitKey(100);
    cout << "ESC = " << int('\033') << " waitKey --> " << code << endl;
    if(code != -1)
      break;
  }

  return 0;
}

