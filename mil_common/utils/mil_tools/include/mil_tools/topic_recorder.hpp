#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <string.h>

namespace mil_tools
{
/*
basic tool to record ONE type of message on ONE topic according to an arbitrary function.

Default behavior (direct implementation) every message on that topic is recordded and put in a bag
(not much didffrent that rosbag)

Designed to be inherited and the Callback function overloaded so that mesages are reorded according to an arbitrary
function
ie:
  time:record every so many seconds(periodic) or location: every 1 m change in location(spacial).

see indyav_path path_recorder.cpp for an implemenation example
*/

template <class MSG>
class TopicRecorder
{
public:
  TopicRecorder(ros::NodeHandle* _nh);
  virtual void CallBack(const MSG& _msg);
  virtual bool Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

protected:
  ros::NodeHandle* nh_;

  ros::Subscriber sub_;

  std::string topic_;

  std::string file_name_;

  ros::ServiceServer enable_service_;

  uint message_count_ = 0;

  bool enabled_ = false;
  rosbag::Bag bag_;
};
}  // namespace mil_tools
#include "../../src/mil_tools/topic_recorder.cpp"
