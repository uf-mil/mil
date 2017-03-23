#include <sub8_perception/start_gate.hpp>

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
  ros::init(argc, argv, "sub8_start_gate");
  ROS_INFO("Initializing node /sub8_start_gate");
  boost::shared_ptr<Sub8StartGateDetector> sub8_start_gates(new Sub8StartGateDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
