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
  // Check for existence of alarm server
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/get", time_out)) << "/alarm/get service not available";
  return;
}

class AlarmTest : public ::testing::Test
{
public:
  ros::NodeHandle nh;
  string alarm_name = "test_alarm";
  string node_name = "test_alarm_client_node";
  string problem_description = "";
  string json_parameters = "json";
  int severity = 5;  // Highest severity
  AlarmProxy pxy {"test_alarm", false, "test_alarm_client_node", "", "json", 5};
  AlarmTest()
  {
  }
};

TEST_F(AlarmTest, alarmProxyTest)
{
  // Test ctor
  auto ctor_err {"AlarmProxy has filed different than what was expected based on value passed to its ctor."};
  EXPECT_STREQ("test_alarm", pxy.alarm_name.c_str()) << ctor_err;
  EXPECT_STREQ("test_alarm_client_node", pxy.node_name.c_str()) << ctor_err;
  EXPECT_STREQ("", pxy.problem_description.c_str()) << ctor_err;
  EXPECT_STREQ("json", pxy.json_parameters.c_str()) << ctor_err;
  EXPECT_EQ(5, pxy.severity) << ctor_err;

  // Test conversion ctor
  EXPECT_EQ(pxy, AlarmProxy(pxy.as_msg()))
    << "Proxy was turned into a ros msg and back into a proxy but is different from original";

  // Test copy ctor
  AlarmProxy pxy2 = pxy;
  
  // Test operator==
  EXPECT_EQ(pxy, pxy2)
    << "Operator == returned false when called w/ the original and a AlarmProxy copy constructed from it";
  AlarmProxy pxy3 = pxy;
  EXPECT_EQ(pxy2, pxy3)
    << "Operator == returned false when called w/ 2 copies of the same AlarmProxy.";
}

TEST_F(AlarmTest, broadcasterTest)
{
  // Construct Kill broadcaster using the 2 different methods
  AlarmBroadcaster a_caster1{nh}; // manipulates internal AlarmProxy that is copy of one in test fixture
  a_caster1.alarm() = pxy;

  AlarmBroadcaster a_caster2{nh, &pxy}; // manipulates external AlarmProxy

  AlarmListener<> listener(nh, alarm_name);  // callback-less listener just for testing broadcasters

  EXPECT_FALSE(listener.is_raised()) << "'test_alarm' should start out cleared";
  EXPECT_EQ(!listener.is_raised(), listener.is_cleared()) << "is_raised() returned the same thing as is_cleared()";

  // Test raising and clearing
  auto raise_msg {"Broadcaster wasn't able to raise test_alarm"};
  auto clear_msg {"Broadcaster wasn't able to clear test_alarm"};
  a_caster1.raise();
  EXPECT_TRUE(listener.query_raised()) << raise_msg;
  a_caster1.clear();
  EXPECT_FALSE(listener.query_raised()) << clear_msg;
  a_caster2.raise();
  EXPECT_TRUE(listener.query_raised()) << raise_msg;
  a_caster2.clear();
  EXPECT_FALSE(listener.query_raised()) << clear_msg;

  // Test manipulating external alarm from internal reference or outside and publishing to server
  pxy.severity = 5;                                    // Change external AlarmProxy
  a_caster2.alarm().raised = true;                     // Change internal handle
  EXPECT_EQ(pxy, a_caster2.alarm());                   // And yet, pxy == a_caster2.alarm()
  a_caster2.publish();                                 // Publish changed status to server
  EXPECT_EQ(a_caster2.alarm(), listener.get_alarm())   // Listener should confirm server received updates
    << a_caster2.alarm().str(true) + " =/= " + listener.get_alarm().str(true);
  EXPECT_EQ(5, listener.get_alarm().severity) << "Unable to use external AlarmProxy to publish new alarm status";

  // Test changing alarm via internal handle
  a_caster1.updateSeverity(3);
  EXPECT_EQ(3, listener.get_alarm().severity);
  a_caster1.alarm().problem_description = "There's no problem here";
  a_caster1.publish();
  EXPECT_STREQ("There's no problem here", listener.get_alarm().problem_description.c_str());
  a_caster1.clear();
  EXPECT_TRUE(listener.query_cleared());

  return;
}

TEST_F(AlarmTest, heartbeatPublisher)
{
  return;
}
