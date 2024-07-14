#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <mil_missions/DoMissionAction.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace mil_missions
{
class MissionClient
{
public:
  MissionClient(ros::NodeHandle &nh);

  // About missions
  std::vector<std::string> available_missions();

  // Execute missions
  bool wait_for_server(const ros::Duration timeout = ros::Duration(5.0));
  bool wait_for_result(const ros::Duration timeout = ros::Duration(0));
  DoMissionResult::ConstPtr get_result();
  actionlib::SimpleClientGoalState get_state();
  void run_mission(std::string mission_name, std::string parameters = "");
  void cancel_mission();

private:
  actionlib::SimpleActionClient<mil_missions::DoMissionAction> ac;
  ros::NodeHandle _nh;
};
}  // namespace mil_missions
// #include "../../src/client.cpp"
