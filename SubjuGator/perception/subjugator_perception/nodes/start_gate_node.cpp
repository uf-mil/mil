#include <subjugator_perception/start_gate.hpp>

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_gate");
  ROS_INFO("Initializing node /start_gate");
  boost::shared_ptr<SubjuGatorStartGateDetector> subjugator_start_gates(new SubjuGatorStartGateDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
