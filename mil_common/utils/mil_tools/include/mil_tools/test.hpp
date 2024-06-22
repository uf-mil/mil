#pragma once

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>

namespace mil_tools
{
bool gazeboModelExists(ros::NodeHandle &nh, const std::string &_name,
                       const ros::WallDuration _timeout = ros::WallDuration(5, 0))
{
  ros::WallTime timeout = ros::WallTime::now() + _timeout;
  while (ros::WallTime::now() < timeout)
  {
    gazebo_msgs::ModelStatesConstPtr modelStates = ros::topic::waitForMessage<gazebo_msgs::ModelStates>(
        std::string("/gazebo/model_states"), nh, ros::Duration(0.3));

    if (!modelStates)
      continue;
    for (auto model : modelStates->name)
    {
      if (model == _name)
        return true;
    }
  }
  return false;
}

gazebo_msgs::ModelState getGazeboModelState(ros::NodeHandle &nh, const std::string &_name,
                                            const ros::WallDuration _timeout = ros::WallDuration(5, 0))
{
  ros::WallTime timeout = ros::WallTime::now() + _timeout;
  while (ros::WallTime::now() < timeout)
  {
    gazebo_msgs::ModelStatesConstPtr modelStates = ros::topic::waitForMessage<gazebo_msgs::ModelStates>(
        std::string("/gazebo/model_states"), nh, ros::Duration(0.3));

    if (!modelStates)
      continue;
    for (size_t i = 0; i < modelStates->name.size(); i++)
    {
      if (modelStates->name[i] == _name)
      {
        gazebo_msgs::ModelState state;
        state.model_name = modelStates->name[i];
        state.pose = modelStates->pose[i];
        state.twist = modelStates->twist[i];
        state.reference_frame = "";
        return state;
      }
    }
  }
  return gazebo_msgs::ModelState();
}
}  // namespace mil_tools
