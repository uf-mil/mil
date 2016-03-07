#ifndef KILL_HANDLING_BROADCASTER_H
#define KILL_HANDLING_BROADCASTER_H

#include <string>

#include <ros/ros.h>

#include "kill_handling/SetKill.h"
#include "kill_handling/Kill.h"

namespace kill_handling {
struct KillBroadcaster {
  ros::NodeHandle nh_;
  std::string id_;
  std::string description_;

  ros::ServiceClient set_kill_;

  KillBroadcaster(std::string id, std::string description) : id_(id), description_(description) {
    set_kill_ = nh_.serviceClient<kill_handling::SetKill>("/set_kill");
  }

  void send(bool active) {
    SetKill k;
    k.request.kill.header.stamp = ros::Time::now();
    k.request.kill.active = active;
    k.request.kill.id = id_;
    k.request.kill.description = description_;
    k.request.clear = false;
    set_kill_.call(k);
  }

  void clear() {
    SetKill k;
    k.request.clear = true;
    k.request.kill.id = id_;
    set_kill_.call(k);
  }
};
}

#endif
