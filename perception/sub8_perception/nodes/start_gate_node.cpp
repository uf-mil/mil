#include <sub8_perception/start_gate.hpp>

#include "ros/ros.h"

#ifdef VISUALIZE
  #warning(Compiling with visualization enabled for start_gate_node)
#else
  #warning(Compiling with NO visualization for start_gate_node)
#endif

int main(int argc, char **argv) {
  ros::init(argc, argv, "sub8_start_gate");
  ROS_INFO("Initializing node /sub8_start_gate");
  boost::shared_ptr<Sub8StartGateDetector> sub8_start_gates(new Sub8StartGateDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
