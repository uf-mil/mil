#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

namespace mil_tools
{
bool gazeboModelExists(ros::NodeHandle &nh, const std::string &_name, const ros::WallDuration _timeout = ros::WallDuration(5, 0))
{
  ros::WallTime timeout = ros::WallTime::now() + _timeout;
  while (ros::WallTime::now() < timeout)
  {
    gazebo_msgs::ModelStatesConstPtr modelStates =
        ros::topic::waitForMessage<gazebo_msgs::ModelStates>(std::string("/gazebo/model_states"), nh, ros::Duration(0.3));

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
}  // namespace mil_tools
