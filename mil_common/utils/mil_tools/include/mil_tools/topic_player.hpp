#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <string.h>

namespace mil_tools
{
/*Utility that publishes the contents of a bag assuming that bag has ONE topic recorded on it.*/
template <class MSG>
class TopicPlayer
{
public:
  TopicPlayer(ros::NodeHandle* _nh);
  virtual bool Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

protected:
  ros::NodeHandle* nh_;

  std::string topic_;

  rosbag::Bag bag_;

  bool enabled_ = false;
  ros::ServiceServer enable_service_;

  ros::Timer start_timer_;
  virtual void Play(const ros::TimerEvent& event);
  ros::Publisher pub_;
};
}  // namespace mil_tools

#include "../../src/mil_tools/topic_player.cpp"
