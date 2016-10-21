#include <pcd_colorizer.hpp>
#include <ros_camera_stream.hpp>
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

  for(string topic : nav::tools::getRectifiedImageTopics())
  {
    cout << topic << endl;
  }
  return 0;
}