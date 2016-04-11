#include <sub8_perception/buoy.hpp>
#include <sub8_pcl/torpedo_board.hpp>

#include "ros/ros.h"

#ifdef VISUALIZE
  #warning(Compiling with visualization enabled)
#else
  #warning(Compiling with NO visualization)
#endif

#ifdef SEGMENTATION_DEBUG
  #warning(Compiling with segmentation debugging info enabled)
#else
  #warning(Compiling with NO segmentation debugging info)
#endif

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_perception");
  ROS_INFO("Initializing node /pcl_perception");
  // boost::shared_ptr<Sub8BuoyDetector> sub8_buoys(new Sub8BuoyDetector());
  Sub8TorpedoBoardDetector sub8_torp_board = Sub8TorpedoBoardDetector();
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
