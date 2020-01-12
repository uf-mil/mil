/**
* Author: David Soto
* Date: Jan 16, 2017
*/
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_alarms_cpp_integration_test");
  return RUN_ALL_TESTS();
}
