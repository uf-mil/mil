/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <functional>
#include <iostream>
#include <ros_alarms/broadcaster.hpp>
#include <ros_alarms/heartbeat_monitor.hpp>
#include <ros_alarms/listener.hpp>

using namespace std;
using ros_alarms::Alarm;
using ros_alarms::AlarmProxy;
using ros_alarms::AlarmListener;
using ros_alarms::AlarmBroadcaster;

TEST(ServerCheck, setServiceCheck)
{
  // Check for existence of alarm server
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::NodeHandle nh;
  ros::Duration time_out{ 2, 0 };
  ASSERT_TRUE(ros::service::waitForService("/alarm/set", time_out)) << "/alarm/set service not available";
  return;
}

TEST(ServerCheck, getServiceCheck)
{
  // Check for existence of alarm server
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::NodeHandle nh;
  ros::Duration time_out{ 2, 0 };
  ASSERT_TRUE(ros::service::waitForService("/alarm/get", time_out)) << "/alarm/get service not available";
  return;
}

TEST(ListenerTest, listenerTest)
{
  ros::NodeHandle nh;
  string alarm_name = "test_alarm";
  AlarmProxy pxy{ "test_alarm", false, "test_alarm_client_node", "", "json", 5 };

  // Create broadcaster & listener
  // You can skip the template args for any
  AlarmListener<> listener{ nh, alarm_name };  // You can omit template args for any cb that can
  EXPECT_TRUE(listener.ok());                  // be put into a std::function<void(AlarmProxy)>
  AlarmBroadcaster ab(nh);
  ab.getAlarm() = pxy;
  ab.clear();  // alarm starts off cleared

  // Last update time happened wehen we called ab.clear()
  auto first_query = listener.getLastUpdateTime();

  ab.updateSeverity(5);  // This is an update to the alarm

  // The listener isn't querying the server before returning the status on these next
  //   two lines, It is returning the current status of the internal AlarmProxy object,
  //   which is not updated by these calls.
  listener.isRaised();
  listener.isCleared();

  // Last update time should not have changed because of calls to isRaised() or isCleared()
  //   unless you are unlucky enough to have an update cb in that short time
  EXPECT_EQ(first_query, listener.getLastUpdateTime());
  EXPECT_EQ(listener.getCachedAlarm().raised, listener.isRaised());

  // The following query_* functions query the server before reporting the status, so the
  //   last update time should have changed
  listener.queryRaised();
  listener.queryCleared();
  EXPECT_NE(first_query, listener.getLastUpdateTime());

#define PRINT(x) cerr << #x ":" << x << endl;  // For debugging in case of test failures
#define COUNTS()                                                                                                       \
  {                                                                                                                    \
    cerr << endl;                                                                                                      \
    PRINT(update_count)                                                                                                \
    PRINT(lo_priority_raise_count)                                                                                     \
    PRINT(hi_priority_raise_count)                                                                                     \
    PRINT(exact_priority_raise_count)                                                                                  \
    PRINT(raise_count)                                                                                                 \
    PRINT(clear_count)                                                                                                 \
  }
  // Setup callback functions
  int update_count = 0;                // any updates
  int lo_priority_raise_count = 0;     // raises w/ priority in [0,2]
  int hi_priority_raise_count = 0;     // raises w/ priority in [4,5]
  int exact_priority_raise_count = 0;  // raises w/ priority of exactly 3
  int raise_count = 0;                 // raises of any priority
  int clear_count = 0;                 // any clears
  function<void(AlarmProxy)>           // All cb funcs for this listener need to have this signature
      update_cb = [&update_count](AlarmProxy pxy) -> void { ++update_count; };
  function<void(AlarmProxy)> lo_raise_cb = [&lo_priority_raise_count](AlarmProxy pxy) -> void {
    ++lo_priority_raise_count;
  };
  function<void(AlarmProxy)> hi_raise_cb = [&hi_priority_raise_count](AlarmProxy pxy) -> void {
    ++hi_priority_raise_count;
  };
  function<void(AlarmProxy)> exact_raise_cb = [&exact_priority_raise_count](AlarmProxy pxy) -> void {
    ++exact_priority_raise_count;
  };
  function<void(AlarmProxy)> raise_cb = [&raise_count](AlarmProxy pxy) -> void { ++raise_count; };
  function<void(AlarmProxy)> clear_cb = [&clear_count](AlarmProxy pxy) -> void { ++clear_count; };

  // Listener will now start processing callbacks from messages to '/alarm/updates'
  EXPECT_TRUE(listener.waitForConnection(ros::Duration(0.4))) << "Timed out trying to detect a publisher to the "
                                                                 "'/alarm/updates' topic";
  ASSERT_GE(listener.getNumConnections(), 1) << "There are no publishers on the '/alarm/updates' topic";
  listener.start();  // Starts spinning in worker thread

  // Make sure initial conditians are good for testing callbacks
  ab.getAlarm().raised = false;
  ab.getAlarm().severity = 0;
  ab.publish();
  ASSERT_FALSE(listener.queryRaised());
  ASSERT_EQ(0, listener.getCachedAlarm().severity);
  ASSERT_EQ(0, update_count);
  ASSERT_TRUE(update_count == lo_priority_raise_count == hi_priority_raise_count == exact_priority_raise_count ==
              raise_count == clear_count);

  // Add callbacks to listener
  listener.clearCallbacks();
  listener.addCb(update_cb);               // Called for any update of the alarm
  listener.addRaiseCb(lo_raise_cb, 0, 2);  // Last 2 args specify severity range
  listener.addRaiseCb(hi_raise_cb, 4, 5);  // Last 2 args specify severity range
  listener.addRaiseCb(exact_raise_cb, 3);  // Use this overload for a single severity
  listener.addRaiseCb(raise_cb);           // Use this overload for any severity raise
  listener.addClearCb(clear_cb);           // Called for any clear of the alarm

  ros::Duration latency(0.01);  // Approximate upper bound on publisher latency
  auto update_start = update_count;
  auto lo_start = lo_priority_raise_count;
  auto hi_start = hi_priority_raise_count;
  auto exact_start = exact_priority_raise_count;
  auto raise_start = raise_count;
  auto clear_start = clear_count;
  for (int i = 0; i <= 5; i++)
  {
    ab.updateSeverity(i);
    ab.clear();
    latency.sleep();
    auto update_diff = update_count - update_start;
    auto lo_diff = lo_priority_raise_count - lo_start;
    auto hi_diff = hi_priority_raise_count - hi_start;
    auto exact_diff = exact_priority_raise_count - exact_start;
    auto raise_diff = raise_count - raise_start;
    auto clear_diff = clear_count - clear_start;

    if (i == 0)
    {
      EXPECT_EQ(2, update_diff);
      EXPECT_EQ(1, lo_diff);
      EXPECT_EQ(1, raise_diff);
      EXPECT_EQ(1, clear_diff);
    }
    else if (i == 1)
    {
      EXPECT_EQ(4, update_diff);
      EXPECT_EQ(2, lo_diff);
      EXPECT_EQ(2, raise_diff);
      EXPECT_EQ(2, clear_diff);
    }
    else if (i == 2)
    {
      EXPECT_EQ(6, update_diff);
      EXPECT_EQ(3, lo_diff);
      EXPECT_EQ(3, raise_diff);
      EXPECT_EQ(3, clear_diff);
    }
    else if (i == 3)
    {
      EXPECT_EQ(8, update_diff);
      EXPECT_EQ(1, exact_diff);
      EXPECT_EQ(4, raise_diff);
      EXPECT_EQ(4, clear_diff);
    }
    else if (i == 4)
    {
      EXPECT_EQ(10, update_diff);
      EXPECT_EQ(1, hi_diff);
      EXPECT_EQ(5, raise_diff);
      EXPECT_EQ(5, clear_diff);
    }
    else if (i == 5)
    {
      EXPECT_EQ(12, update_diff);
      EXPECT_EQ(3, lo_diff);
      EXPECT_EQ(2, hi_diff);
      EXPECT_EQ(1, exact_diff);
      EXPECT_EQ(6, raise_diff);
      EXPECT_EQ(6, clear_diff);
    }
  }
  return;
}
