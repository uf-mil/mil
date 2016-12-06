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

  ros::NodeHandle nh;
  string challenge_name = "underwater_shape_identification";
  ros::init(argc, argv, challenge_name + "_vision");
  string name_space{challenge_name + "/"};

  int img_buffer_size = 0;
  nh.param<int>(name_space + "buffer_size", img_buffer_size, 5);
  nav::UnderwaterShapeDetector underwater_shape_detector(nh, img_buffer_size, name_space);

  return 0;
}

