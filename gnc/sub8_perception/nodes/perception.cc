#include <sub8_pcl/buoy.hpp>
#include <sub8_pcl/torpedo_board.hpp>

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_slam");
  ROS_INFO("Initializing node /pcl_slam");
  boost::shared_ptr<Sub8BuoyDetector> sub8_buoys(new Sub8BuoyDetector());
  Sub8TorpedoBoardDetector sub8_torp_board = Sub8TorpedoBoardDetector();
  ROS_INFO("Spinning ros callbacks");
  ros::spin();
}