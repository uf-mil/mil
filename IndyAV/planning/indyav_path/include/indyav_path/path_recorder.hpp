#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mil_tools/mil_tools.hpp>

template <class ROS_TIME, class ROS_DURATION>
class PathRecorder : protected mil_tools::TopicRecorder<nav_msgs::Odometry>
{
private:
  ROS_DURATION period_;
  ROS_TIME last_save_;

public:
  PathRecorder(ros::NodeHandle* _nh) : mil_tools::TopicRecorder<nav_msgs::Odometry>(_nh)
  {
    double rate;
    if (!_nh->getParam("record_rate", rate))
    {
      ROS_FATAL("path_recorder, no rate specified");
    }
    period_ = ROS_DURATION(1.0 / rate);
    last_save_ = ROS_TIME::now();
  }
  // TODO: findout what the controller actually needs and place it here
  void CallBack(const nav_msgs::Odometry& _msg)
  {
    if (!enabled_)
    {
      // check frame id from before be start
      if (_msg.header.frame_id != "/ecef")
        ROS_WARN("path_recorder frame id is not in ECEF");
    }
    else
    {
      auto now = ROS_TIME::now();
      auto next = last_save_ + period_;
      if (next >= now)
        return;
      ++message_count_;
      bag_.write(topic_, ros::Time(now.sec, now.nsec), _msg);
      last_save_ = now;
    }
  }
};
