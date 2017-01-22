#include <ros_alarms/listener.hpp>
#include <ros_alarms/broadcaster.hpp>
#include <ros_alarms/heartbeat_monitor.hpp>
#include <iostream>
#include <gtest/gtest.h>

using namespace std;
using ros_alarms::AlarmProxy;
using ros_alarms::AlarmListener;
using ros_alarms::AlarmBroadcaster;
using ros_alarms::HeartbeatMonitor;


TEST(ServerCheck, setServiceCheck)
{
  // Check for existence of alarm server
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/set", time_out)) << "/alarm/set service not available";
  
  return;
}

TEST(ServerCheck, getServiceCheck)
{
  // Check for existence of alarm server
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/get", time_out)) << "/alarm/get service not available";
  return;
}

class AlarmTest : public ::testing::Test
{
public:
  AlarmTest() : pxy(alarm_name, node_name, problem_description, json_parameters, severity)
  ros::NodeHandle nh;
  string alarm_name = "test_alarm";
  string node_name = "test_alarm_client_node";
  string problem_description = "";
  string json_parameters = "json";
  int severity = 5;  // Highest severityj
  AlarmProxy pxy;
};

TEST_F(AlarmTest, alarmProxyTest)
{
  EXPECT_STREQ(alarm_name.c_str(), "test_alarm");
  EXPECT_STREQ(node_name.c_str(), "test_alarm_client_node");
  EXPECT_STREQ(problem_description.c_str(), "");
  EXPECT_STREQ(json_parameters.c_str(), "json");
  EXPECT_EQ(severity, 5);
  EXPECT_EQ(pxy, AlarmProxy(pxy.as_msg()));
}

TEST(AlarmTest, broascasterListenerTest)
{

  // Alarm Proxy struct for interfacing with broadcasters and listeners
  ros_alarms::AlarmProxy alarm_px(alarm_name, 
  // Instantiate an alarm broadcaster
  AlarmBroadcaster bcast{nh, alarm_px};
  return;
}

TEST(AlarmTest, heartbeatPublisher)
{
  return;
}
