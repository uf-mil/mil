/**
 * Author: Cameron Brown
 * Date: June 1, 2024
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mil_tools/test.hpp>

using namespace mil_tools;

// Ensure that at least sub8 or sub9 shows up when launching gazebo
TEST(SpawnTest, spawnTest)
{
  ros::NodeHandle nh;
  EXPECT_TRUE(mil_tools::gazeboModelExists(nh, "sub8") || mil_tools::gazeboModelExists(nh, "sub8_no_cams") || mil_tools::gazeboModelExists(nh, "sub9"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subjugator_gazebo_spawn_test");
  return RUN_ALL_TESTS();
}
