#include <gtest/gtest.h>
#include <mil_missions/client.h>
#include <ros/ros.h>
#include <iostream>

#include <mil_missions/DoMissionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace mil_missions;

class MissionClientTest : public ::testing::Test
{
protected:
  MissionClientTest() : client(nh) {}

  ros::NodeHandle nh;
  mil_missions::MissionClient client;
};

TEST_F(MissionClientTest, RunMission)
{
  EXPECT_TRUE(client.wait_for_server());

  // Test running a mission
  client.run_mission("PrintAndWait");
  EXPECT_TRUE(client.wait_for_result(ros::Duration(5.0)));
  mil_missions::DoMissionResult::ConstPtr result = client.get_result();
  actionlib::SimpleClientGoalState state = client.get_state();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(result->success);
  EXPECT_EQ(result->parameters, "");
  EXPECT_EQ(result->result, "The darkness isn't so scary");
}

TEST_F(MissionClientTest, FailingMission) {
  EXPECT_TRUE(client.wait_for_server());

  // Test running a mission
  client.run_mission("FailingMission");
  EXPECT_TRUE(client.wait_for_result(ros::Duration(5.0)));
  mil_missions::DoMissionResult::ConstPtr result = client.get_result();
  actionlib::SimpleClientGoalState state = client.get_state();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(result->success);
  EXPECT_TRUE(!result->parameters.empty());
  EXPECT_EQ(result->result, "Exception occurred in FailingMission mission: ValueError: This is an example error!");
}

TEST_F(MissionClientTest, CancelledTest) {
    EXPECT_TRUE(client.wait_for_server());
    
    // Test running a mission
    client.run_mission("CancelledMission");
    EXPECT_TRUE(client.wait_for_result(ros::Duration(5.0)));
    mil_missions::DoMissionResult::ConstPtr result = client.get_result();
    actionlib::SimpleClientGoalState state = client.get_state();
    EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);
    EXPECT_FALSE(result->success);
    EXPECT_TRUE(result->parameters.empty());
    EXPECT_EQ(result->result, "mission cancelled");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "run_mission_test_cpp");
  return RUN_ALL_TESTS();
}
