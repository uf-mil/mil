#include <sub8_perception/start_gate.hpp>

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sub8_start_gate");
  ROS_INFO("Initializing node /sub8_start_gate");
  boost::shared_ptr<Sub8StartGateDetector> sub8_start_gates(new Sub8StartGateDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
