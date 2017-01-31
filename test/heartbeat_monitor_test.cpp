#include <ros_alarms/listener.hpp>
#include <ros_alarms/broadcaster.hpp>
#include <ros_alarms/heartbeat_monitor.hpp>
#include <std_msgs/String.h>
#include <iostream>
#include <functional>
#include <gtest/gtest.h>

using namespace std;
using ros_alarms::Alarm;
using ros_alarms::AlarmProxy;
using ros_alarms::AlarmListener;
using ros_alarms::AlarmBroadcaster;
using ros_alarms::HeartbeatMonitor;

TEST(HeartbeatMonitorTest, heartbeatMonitorTest)
{
  ros::NodeHandle nh;
  string heartbeat_topic {"heartbeat_topic"};
  string alarm_name {"lost_heartbeat"};
  ros::Duration time_to_raise(0.2);
  ros::Duration time_to_clear(0.4);

  // Publish heartbeat
  ros::Publisher heartbeat_pub = nh.advertise<std_msgs::String>(heartbeat_topic, 1000);
  auto pub_valid = [&heartbeat_pub](bool valid){ // Can publish a valid or invalid heartveat
    std_msgs::String msg;
    msg.data = (valid? "Will pass the predicate" : ""); // second one will
    heartbeat_pub.publish(msg); };

  // Listener to get status of alarm
  AlarmListener<> listener{nh, alarm_name};

  // Create callbacks
  function<bool(std_msgs::String)> predicate = [](std_msgs::String msg)
  {
   return (msg.data.size() == 0? false : true);
  };

  // Initialize HertbeatMonitor with topic name, alarm name, period
  HeartbeatMonitor<std_msgs::String>
    hb_monitor(nh, alarm_name, heartbeat_topic, predicate, time_to_raise, time_to_clear);
  EXPECT_TRUE(listener.ok());
  EXPECT_TRUE(listener.query_cleared()) << "The alarm should start out cleared";
  EXPECT_STREQ(alarm_name.c_str(), hb_monitor.alarm_name().c_str());
  EXPECT_STREQ(heartbeat_topic.c_str(), hb_monitor.heartbeat_name().c_str());


  EXPECT_GE(1, hb_monitor.getNumConnections())
    << "Monitor should be connected to at least one publisher";
  ASSERT_TRUE(hb_monitor.waitForConnection(ros::Duration(0.5)))  // Returns false if timed-out
    << "Timed out waiting for a publisher to " << heartbeat_topic;

  // Neede to test time_to_raise and time_to_clear behaviour
  ros::Time monitor_start_time = ros::Time::now();
  hb_monitor.startMonitoring();

  // Timing utility function for this test
  auto sleep_until = [](ros::Duration offset, ros::Time reference)
  {
    auto sleep_time = reference + offset - ros::Time::now();
    return sleep_time > ros::Duration(0.0)? sleep_time.sleep() : false;
  };

  sleep_until(time_to_raise * 0.8, monitor_start_time);
  EXPECT_TRUE(hb_monitor.healthy())
    << "Heartbeat should be healthy before the loss time threshold {time_to_raise} is cleared.";
  listener.waitForUpdate();
  EXPECT_TRUE(listener.query_cleared());
  sleep_until(time_to_raise * 1.2, monitor_start_time);
  EXPECT_FALSE(hb_monitor.healthy())
    << "Heartbeat shouldn't be healthy after {time_to_raise} w/o receiving a heartbeat.";
  listener.waitForUpdate();
  EXPECT_TRUE(listener.query_raised());

  auto recovery_start_time = ros::Time::now();
  pub_valid(true);
  while(ros::Time::now() - recovery_start_time < time_to_clear) // Heartbeat is recovering here
  {
    sleep_until(time_to_raise * 0.8, ros::Time::now());
    pub_valid(true);
  }
  EXPECT_TRUE(listener.query_cleared())
    << "Monitor has gotten beats for over {time_to_clear}, alarm should have cleared.";
  
  sleep_until(time_to_raise * 1.2, hb_monitor.last_beat());
  EXPECT_TRUE(listener.query_raised())
    << "Heartbeat shouldn't be healthy after {time_to_raise} w/o receiving a heartbeat.";

  while(ros::Time::now() - hb_monitor.last_beat() < time_to_clear) // Shouldn't recover here,
  {                                                                //  invalid heartbeat
    sleep_until(time_to_raise * 0.8, ros::Time::now());
    pub_valid(false);  // False --> publish invalid heartbeat
  }
  EXPECT_TRUE(listener.query_raised());

  auto ref_time = ros::Time::now();
  while(ros::Time::now() - ref_time < time_to_clear*2.0) // Given a lot of time to recover,
  {                                                      // but never doing so because hb's
    sleep_until(time_to_raise * 1.2, ros::Time::now());  // are too far apart
    pub_valid(true);
  }
  EXPECT_TRUE(listener.query_raised());

  return;
}
