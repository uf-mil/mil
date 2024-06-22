#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/ModelState.h>
#include <gtest/gtest.h>
#include <mil_missions/DoMissionResult.h>
#include <mil_missions/client.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <mil_tools/test.hpp>
#include <ros_alarms/broadcaster.hpp>
#include <ros_alarms/listener.hpp>

using namespace mil_missions;
using namespace mil_tools;

class MissionClientTest : public ::testing::Test
{
protected:
  MissionClientTest() : client(nh)
  {
  }

  ros::NodeHandle nh;
  mil_missions::MissionClient client;
};

TEST_F(MissionClientTest, RunMission)
{
  ASSERT_TRUE(client.wait_for_server());
  ASSERT_TRUE(mil_tools::gazeboModelExists(nh, "start_gate_2022"));
  ASSERT_TRUE(mil_tools::gazeboModelExists(nh, "sub8"));

  // Clear kill
  ros_alarms::AlarmProxy pxy{ "kill", false, nh.getNamespace(), "Unkilling to run mission", "json", 5 };
  ros_alarms::AlarmBroadcaster alarmer{ nh, &pxy };
  alarmer.clear();
  ros_alarms::AlarmListener<> listener(nh, "kill");  // Callback-less listener for testing broadcasters
  ASSERT_TRUE(!listener.isRaised());

  // Test running a mission
  client.run_mission("StartGate2022");

  ros::Duration(7.0).sleep();

  // Lambda, are we in start gate?
  auto in_start_gate = [&]()
  {
    // start gate range:
    // 4.2 < x < 4.8
    // -1.1 < y < 1.1
    gazebo_msgs::ModelState state = mil_tools::getGazeboModelState(nh, "sub8");
    return state.pose.position.x > 4.2 && state.pose.position.x < 4.8 && state.pose.position.y > -1.1 &&
           state.pose.position.y < 1.1;
  };

  // 1. Ensure that we are not within the start gate right now
  ASSERT_TRUE(!in_start_gate());

  actionlib::SimpleClientGoalState action_state = client.get_state();
  std::cout << action_state.state_ << std::endl;
  ASSERT_EQ(action_state, actionlib::SimpleClientGoalState::ACTIVE);

  // Send the side
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/getside", 1);
  std_msgs::Bool msg;
  msg.data = false;
  pub.publish(msg);

  // 2. Ensure that we become within the start gate within 10 seconds
  ASSERT_TRUE(
      [&]()
      {
        ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(10.0);
        while (ros::WallTime::now() < timeout)
        {
          if (in_start_gate())
            return true;
          ros::Duration(0.1).sleep();
        }
        return false;
      }());

  // 3. Check that we get style points (rotation is 720 deg)
  bool started = false;
  bool rotating = false;
  tf2::Quaternion orientation;
  tf2::fromMsg(mil_tools::getGazeboModelState(nh, "sub8").pose.orientation, orientation);
  double angle = 0;
  double total_rotation = 0;

  while (!started || rotating)
  {
    gazebo_msgs::ModelState state = mil_tools::getGazeboModelState(nh, "sub8");
    rotating = state.twist.angular.z > 0.1;
    if (rotating)
      started = false;
    if (!rotating || !in_start_gate())
      break;
    tf2::fromMsg(state.pose.orientation, orientation);
    total_rotation += tf2::getYaw(state.pose.orientation) - angle;
    angle = tf2::getYaw(state.pose.orientation);
  }
  // Ideal: 6.28 ~= 2*pi (720 deg)
  ASSERT_TRUE((total_rotation >= 6) && (total_rotation <= 7));

  // 3. Ensure that we are not within the start gate within 10 seconds
  EXPECT_TRUE(
      [&]()
      {
        ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(10.0);
        while (ros::WallTime::now() < timeout)
        {
          if (!in_start_gate())
            return true;
          ros::Duration(0.1).sleep();
        }
        return false;
      }());

  mil_missions::DoMissionResult::ConstPtr result = client.get_result();
  actionlib::SimpleClientGoalState state = client.get_state();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(result->success);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "start_gate_test");
  return RUN_ALL_TESTS();
}
