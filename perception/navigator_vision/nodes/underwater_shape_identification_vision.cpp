#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <navigator_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <navigator_vision_lib/image_filtering.hpp>

using namespace std;

int main(int argc, char** argv) 
{ 
  cout << "\033[1;31mUnderwater Shape Identification Vision\033[0m" << endl;  
  int rotations = stoi(argv[2]);

  // set up image acquisition
  string cam_topic {"down/image_raw"};
  size_t history_length{200};
  ros::init(argc, argv, "underwater_shape_identification_vision");
  ros::NodeHandle nh;

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

  // Test kernel rotation and averaging
  auto file_name = "/home/santiago/mil_ws/src/Navigator/perception/navigator_vision/templates/" + string(argv[1]) + ".png";
  cv::Mat kernel = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
  if(!kernel.data)
  {
    cout << "Could not load image from " << file_name << endl;
    return -1;
  }

  cv::Mat rot_invar_kernel;
  makeRotInvariant(kernel, rotations).convertTo(rot_invar_kernel, CV_8UC1);
  cv::imshow("rot invariant kernel", rot_invar_kernel);
  cv::waitKey(0);
  return 0;
}

