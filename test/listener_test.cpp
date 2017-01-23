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
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/set", time_out)) << "/alarm/set service not available";
  
  return;
}

TEST(ServerCheck, getServiceCheck)
{
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  // Check for existence of alarm server
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/get", time_out)) << "/alarm/get service not available";
  return;
}

class AlarmTest : public ::testing::Test
{
public:
  AlarmTest() : pxy(alarm_name, false, node_name, problem_description, json_parameters, severity) 
  {}
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
  auto ctor_err {"AlarmProxy has filed different than what was wxpected based on value passed to its ctor."};
  EXPECT_STREQ(alarm_name.c_str(), "test_alarm") << ctor_err;
  EXPECT_STREQ(node_name.c_str(), "test_alarm_client_node") << ctor_err;
  EXPECT_STREQ(problem_description.c_str(), "") << ctor_err;
  EXPECT_STREQ(json_parameters.c_str(), "json") << ctor_err;
  EXPECT_EQ(severity, 5) << ctor_err;
  EXPECT_EQ(pxy, AlarmProxy(pxy.as_msg()))
    << "Proxy was turned into a ros msg and back into a proxy but is different from original";
  AlarmProxy pxy2 = pxy;  // Test copy ctor & operator ==
  EXPECT_EQ(pxy, pxy2)
    << "Operator == returned false when called w/ the original and a AlarmProxy copy constructed from it";
  AlarmProxy pxy3 = pxy;
  EXPECT_EQ(pxy2, pxy3)
    << "Operator == returned false when called w/ 2 copies of the same AlarmProxy.";
}

TEST_F(AlarmTest, broascasterListenerTest)
{
  return;
}

TEST_F(AlarmTest, heartbeatPublisher)
{
  return;
}
