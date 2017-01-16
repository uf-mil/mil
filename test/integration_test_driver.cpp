/**
* Author: David Soto
* Date: 1/15/17
*/
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_alarms_cpp_integration_test");
  std::cerr << "TESTTING CPP ALARMS!!" << std::endl;
  return RUN_ALL_TESTS();
}
