/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#include <gtest/gtest.h>
#include <ros_alarms/alarm_proxy.hpp>

using namespace std;
using ros_alarms::Alarm;
using ros_alarms::AlarmProxy;

TEST(AlarmProxyTest, alarmProxyTest)
{
  AlarmProxy pxy{ "test_alarm", false, "test_alarm_client_node", "", "json", 5 };

  // Test ctor
  const char* ctor_err = "AlarmProxy different field than expected based on value passed to ctor.";
  EXPECT_STREQ("test_alarm", pxy.alarm_name.c_str()) << ctor_err;
  EXPECT_STREQ("test_alarm_client_node", pxy.node_name.c_str()) << ctor_err;
  EXPECT_STREQ("", pxy.problem_description.c_str()) << ctor_err;
  EXPECT_STREQ("json", pxy.json_parameters.c_str()) << ctor_err;
  EXPECT_EQ(5, pxy.severity) << ctor_err;

  // Test conversion ctor
  EXPECT_EQ(pxy, AlarmProxy(pxy.as_msg())) << "Proxy was turned into a ros msg and back into a proxy but is different "
                                              "from original";

  // Test operator== and copy cotr
  AlarmProxy pxy2 = pxy;
  EXPECT_EQ(pxy, pxy2) << "Operator == returned false when called w/ an AlarmProxy copy constructed from itself";
  AlarmProxy pxy3 = pxy;
  EXPECT_EQ(pxy2, pxy3) << "Operator == returned false when called w/ 2 copies of the same AlarmProxy.";
}
