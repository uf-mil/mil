#ifndef KILL_HANDLING_LISTENER_H
#define KILL_HANDLING_LISTENER_H

#include <string>
#include <vector>
#include <map>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>

#include "kill_handling/KillsStamped.h"
#include "kill_handling/Kill.h"

namespace kill_handling {

struct KillListener {
  void _killmsg_callback(const KillsStampedConstPtr &msg) {
    if (!_kills) {
      // Initialize _kills map
      _kills = std::map<std::string, Kill>();
    } else {
      // Clear the old kills
      _kills->clear();
    }

    // Copy the new kill msg into _kills
    BOOST_FOREACH (const Kill &kill, msg->kills) { (*_kills)[kill.id] = kill; }

    _check_killed();
  }

  // Standard method to check if killed and call callbacks if appropriate
  void _check_killed() {
    bool killed = get_killed();
    if (killed && !_previously_killed) {
      if (_killed_callback) _killed_callback();  // Just now killed
    } else if (!killed && _previously_killed) {
      if (_unkilled_callback) _unkilled_callback();  // Just now unkilled
    }
    _previously_killed = killed;
  }

  ros::NodeHandle nh;
  boost::function<void()> _killed_callback;
  boost::function<void()> _unkilled_callback;
  boost::optional<std::map<std::string, Kill> > _kills;
  ros::Subscriber _sub;
  bool _previously_killed;
  ros::Timer _check_killed_timer;  // Not implemented

  // Constructor with killed callback and unkilled callback
  KillListener(boost::function<void()> killed_callback = boost::function<void()>(),
               boost::function<void()> unkilled_callback = boost::function<void()>()) {
    _killed_callback = killed_callback;
    _unkilled_callback = unkilled_callback;

    _sub = nh.subscribe<KillsStamped>("/kill", 1,
                                      boost::bind(&KillListener::_killmsg_callback, this, _1));
    _previously_killed = false;

    _check_killed_timer =
        nh.createTimer(ros::Duration(1), boost::bind(&KillListener::_timer_cb, this, _1));
  }

  // User function to see what is causing the kill
  std::vector<std::string> get_kills() {
    std::vector<std::string> res;
    if (_kills) {
      BOOST_FOREACH (const Kill &kill, *_kills | boost::adaptors::map_values) {
        if (kill.active) {
          res.push_back(kill.description);
        }
      }
    }
    return res;
  }

  // Iterate through _kills to find any active kills
  //        If _kills has not been initialized (kill master has not published any kill msgs yet)
  //return true
  bool get_killed() {
    // Check if kill_master is publishing
    if (_sub.getNumPublishers() == 0) {
      // Kill_master isn't publishing
      if (!_kills) {
        // Initialize _kills map
        _kills = std::map<std::string, Kill>();
      } else {
        // Clear the old kills
        _kills->clear();
      }

      // Add a kill to _kills
      Kill k;
      k.id = "This kill listener";
      k.active = true;
      k.description = "Kill master is not publishing";
      (*_kills)["This kill listener"] = k;
      return true;
    }

    if (_kills) {
      BOOST_FOREACH (const Kill &kill, *_kills | boost::adaptors::map_values) {
        if (kill.active) {
          return true;
        }
      }
      return false;
    } else {
      return true;
    }
  }

  void _timer_cb(const ros::TimerEvent &event) { _check_killed(); }
};
}

#endif
