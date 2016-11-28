#include <navigator_vision_lib/colorizer/pcd_colorizer.hpp>
#include <image_acquisition/ros_camera_stream.hpp>
#include <navigator_tools.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>


using namespace std;

int main(int argc, char** argv)
{
  // Init ROS
  ros::init(argc, argv, "velodyne_pcd_colorizer");
  ros::NodeHandle nh;

  // Create PcdColorizer active object
  nav::PcdColorizer colorizer{nh, "/velodyne_points"};

  while(colorizer.ok())
  {
  	ROS_INFO("velodyne_pcd_colorizer: Spinning ROS callbacks");
  	ros::spin();
  }

}