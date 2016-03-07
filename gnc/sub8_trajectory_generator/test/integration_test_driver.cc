/**
* Author: Patrick Emami
* Date: 11/24/15
*/
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tgen_integration_test");
  return RUN_ALL_TESTS();
}