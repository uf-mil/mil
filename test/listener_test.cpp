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

TEST(ServerCheck, setServiceCheck)
{
  // Check for existence of alarm server
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/set", time_out))
    << "/alarm/set service not available";
  
  return;
}

TEST(ServerCheck, getServiceCheck)
{
  // Check for existence of alarm server
  ros::Time::init();  // needed to use ros::Time::now() w/o creating a  NodeHandle
  ros::Duration time_out {2, 0};
  ASSERT_TRUE(ros::service::waitForService("/alarm/get", time_out))
    << "/alarm/get service not available";
  return;
}

class AlarmTest : public ::testing::Test
{
public:
  ros::NodeHandle nh;
  string alarm_name = "test_alarm";
  string node_name = "test_alarm_client_node";
  AlarmProxy pxy {"test_alarm", false, "test_alarm_client_node", "", "json", 5};
  AlarmTest()
  {
  }
};

TEST_F(AlarmTest, alarmProxyTest)
{
  // Test ctor
  auto ctor_err {"AlarmProxy different field than expected based on value passed to ctor."};
  EXPECT_STREQ("test_alarm", pxy.alarm_name.c_str()) << ctor_err;
  EXPECT_STREQ("test_alarm_client_node", pxy.node_name.c_str()) << ctor_err;
  EXPECT_STREQ("", pxy.problem_description.c_str()) << ctor_err;
  EXPECT_STREQ("json", pxy.json_parameters.c_str()) << ctor_err;
  EXPECT_EQ(5, pxy.severity) << ctor_err;

  // Test conversion ctor
  EXPECT_EQ(pxy, AlarmProxy(pxy.as_msg()))
    << "Proxy was turned into a ros msg and back into a proxy but is different from original";

  // Test operator== and copy cotr
  AlarmProxy pxy2 = pxy;
  EXPECT_EQ(pxy, pxy2)
    << "Operator == returned false when called w/ an AlarmProxy copy constructed from itself";
  AlarmProxy pxy3 = pxy;
  EXPECT_EQ(pxy2, pxy3)
    << "Operator == returned false when called w/ 2 copies of the same AlarmProxy.";
}

TEST_F(AlarmTest, broadcasterTest)
{
  // Construct Kill broadcaster using the 2 different methods
  AlarmBroadcaster a_caster1{nh};            // Manipulates encapsulated AlarmProxy that
  a_caster1.alarm() = pxy;                   //   is a copy of the one in the text fixture
  AlarmBroadcaster a_caster2{nh, &pxy};      // Broadcaster is bound to external AlarmProxy
  AlarmListener<> listener(nh, alarm_name);  // Callback-less listener for testing broadcasters

  a_caster1.clear();
  EXPECT_FALSE(listener.query_raised()) << "'test_alarm' should start out cleared";
  EXPECT_EQ(!listener.is_raised(), listener.is_cleared())
    << "is_raised() returned the same thing as is_cleared()";

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

  // Test changing alarm via outside pxy or internal ref to it and updating server
  pxy.severity = 2;                                    // Change external AlarmProxy
  a_caster2.alarm().raised = true;                     // Change internal handle
  EXPECT_EQ(pxy, a_caster2.alarm());                   // And yet, pxy == a_caster2.alarm()
  a_caster2.publish();                                 // Publish changed status to server
  EXPECT_EQ(a_caster2.alarm(), listener.get_alarm())   // Listener should confirm server 
                                                       //   received updates
    << a_caster2.alarm().str(true) + " =/= " + listener.get_alarm().str(true);
  EXPECT_EQ(2, listener.get_alarm().severity)
    << "Unable to use external AlarmProxy to publish new alarm status";

  // Test changing alarm via reference to internal pxy and updating server
  a_caster1.updateSeverity(3);
  EXPECT_EQ(3, listener.get_alarm().severity);
  a_caster1.alarm().problem_description = "There's no problem here";
  a_caster1.publish();
  EXPECT_STREQ("There's no problem here", listener.get_alarm().problem_description.c_str());
  a_caster1.clear();  // make sure alarm starts cleared for following tests
  EXPECT_TRUE(listener.query_cleared());

  return;
}

TEST_F(AlarmTest, listenerTest)
{
  // Create broadcaster & listener
  // You can skip the template args for any
  AlarmListener<> listener{nh, alarm_name};  // You can omit template args for any cb that can
  EXPECT_TRUE(listener.ok());                // be put into a std::function<void(AlarmProxy)>
  AlarmBroadcaster ab(nh);
  ab.alarm() = pxy;
  ab.clear();  // alarm starts off cleared

  // Last update time happened wehen we called ab.clear()
  auto first_query = listener.last_update_time();

  ab.updateSeverity(5); // This is an update to the alarm

  // The listener isn't querying the server before returning the status on these next
  //   two lines, It is returning the current status of the internal AlarmProxy object,
  //   which is not updated by these calls.
  listener.is_raised();
  listener.is_cleared();

  // Last update time should not have changed because of calls to is_raised() or is_cleared()
  //   unless you are unlucky enough to have an update cb in that short time
  EXPECT_EQ(first_query, listener.last_update_time());
  EXPECT_EQ(listener.get_cached_alarm().raised, listener.is_raised());

  // The following query_* functions query the server before reporting the status, so the
  //   last update time should have changed
  listener.query_raised();
  listener.query_cleared();
  EXPECT_NE(first_query, listener.last_update_time());

#define PRINT(x) cerr << #x ":" << x  << endl; // For debugging in case of test failures
#define COUNTS() {                  \
  cerr << endl;                     \
  PRINT(update_count)               \
  PRINT(lo_priority_raise_count)    \
  PRINT(hi_priority_raise_count)    \
  PRINT(exact_priority_raise_count) \
  PRINT(raise_count)                \
  PRINT(clear_count)                \
  }
  // Setup callback functions
  int update_count= 0;                  // any updates
  int lo_priority_raise_count = 0;      // raises w/ priority in [0,2]
  int hi_priority_raise_count = 0;      // raises w/ priority in [4,5]
  int exact_priority_raise_count = 0;   // raises w/ priority of exactly 3
  int raise_count = 0;                  // raises of any priority
  int clear_count = 0;                  // any clears
  function<void(AlarmProxy)>  // All cb funcs for this listener need to have this signature
    update_cb = [&update_count](AlarmProxy pxy) -> void { ++update_count; };
  function<void(AlarmProxy)> lo_raise_cb  =
    [&lo_priority_raise_count ](AlarmProxy pxy) -> void { ++lo_priority_raise_count; };
  function<void(AlarmProxy)> hi_raise_cb  =
    [&hi_priority_raise_count ](AlarmProxy pxy) -> void { ++hi_priority_raise_count; };
  function<void(AlarmProxy)> exact_raise_cb  =
    [&exact_priority_raise_count ](AlarmProxy pxy) -> void { ++exact_priority_raise_count; };
  function<void(AlarmProxy)> raise_cb =
    [&raise_count](AlarmProxy pxy) -> void { ++raise_count; };
  function<void(AlarmProxy)> clear_cb =
    [&clear_count](AlarmProxy pxy) -> void { ++clear_count; };

  // Listener will now start processing callbacks from messages to '/alarm/updates'
  EXPECT_TRUE(listener.wait_for_connection(ros::Duration(0.4)))
    << "Timed out trying to detect a publisher to the '/alarm/updates' topic";
  ASSERT_GE(listener.get_num_connections(), 1)
    << "There are no publishers on the '/alarm/updates' topic";
  listener.start();  // Starts spinning in worker thread

  // Make sure initial conditians are good for testing callbacks
  ab.alarm().raised = false;
  ab.alarm().severity = 0;
  ab.publish();
  ASSERT_FALSE(listener.query_raised());
  ASSERT_EQ(0, listener.get_cached_alarm().severity);
  ASSERT_EQ(0, update_count);
  ASSERT_TRUE(update_count == lo_priority_raise_count == hi_priority_raise_count
    == exact_priority_raise_count == raise_count == clear_count);

  // Add callbacks to listener
  listener.clear_callbacks();
  listener.add_cb(update_cb);                // Called for any update of the alarm
  listener.add_raise_cb(lo_raise_cb, 0, 2);  // Last 2 args specify severity range
  listener.add_raise_cb(hi_raise_cb, 4, 5);  // Last 2 args specify severity range
  listener.add_raise_cb(exact_raise_cb, 3);  // Use this overload for a single severity
  listener.add_raise_cb(raise_cb);           // Use this overload for any severity raise
  listener.add_clear_cb(clear_cb);           // Called for any clear of the alarm

  ros::Duration latency(0.01);  // Approximate upper bound on publisher latency
  auto update_start = update_count;
  auto lo_start = lo_priority_raise_count;
  auto hi_start = hi_priority_raise_count;
  auto exact_start = exact_priority_raise_count;
  auto raise_start = raise_count;
  auto clear_start = clear_count;
  for(int i = 0; i <= 5; i++)
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

    if(i == 0)
    {
      EXPECT_EQ(2, update_diff);
      EXPECT_EQ(1, lo_diff);
      EXPECT_EQ(1, raise_diff);
      EXPECT_EQ(1, clear_diff);
    }
    else if(i == 1)
    {
      EXPECT_EQ(4, update_diff);
      EXPECT_EQ(2, lo_diff);
      EXPECT_EQ(2, raise_diff);
      EXPECT_EQ(2, clear_diff);
    }
    else if(i == 2)
    {
      EXPECT_EQ(6, update_diff);
      EXPECT_EQ(3, lo_diff);
      EXPECT_EQ(3, raise_diff);
      EXPECT_EQ(3, clear_diff);
    }
    else if(i == 3)
    {
      EXPECT_EQ(8, update_diff);
      EXPECT_EQ(1, exact_diff);
      EXPECT_EQ(4, raise_diff);
      EXPECT_EQ(4, clear_diff);
    }
    else if(i == 4)
    {
      EXPECT_EQ(10, update_diff);
      EXPECT_EQ(1, hi_diff);
      EXPECT_EQ(5, raise_diff);
      EXPECT_EQ(5, clear_diff);
    }
    else if(i == 5)
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

TEST_F(AlarmTest, heartbeatMonitorTest)
{
  string heartbeat_topic {"heartbeat_topic"};
  string alarm_name {"lost_heartbeat"};
  ros::Duration time_to_raise(0.2);
  ros::Duration time_to_clear(0.4);
  ros::Duration init_period(0.8);

  // Publish heartbeat
  ros::Publisher heartbeat_pub = nh.advertise<std_msgs::String>(heartbeat_topic, 1000);
  auto pub_valid = [&heartbeat_pub](bool valid){
    std_msgs::String msg;
    msg.data = (valid? "" : "Won't pass the predicate");
    heartbeat_pub.publish(msg); };

  // Listener to get status of alarm
  AlarmListener<> listener{nh, alarm_name};

  // Create callbacks
  function<bool(std_msgs::String)> cb1 =
    [](std_msgs::String msg){ return (msg.data.size() == 0? true : false); };

  // Initialize HertbeatMonitor with topic name, alarm name, period
  ros::Time approx_init_time = ros::Time::now();
  HeartbeatMonitor<std_msgs::String> hb_monitor(nh, alarm_name, heartbeat_topic,
                                                cb1, time_to_raise, time_to_clear,
                                                init_period);
  EXPECT_TRUE(listener.ok());

  // Test for proper initialization
  auto sleep_until = [approx_init_time](double secs_after_init)
    { (approx_init_time + ros::Duration(secs_after_init) - ros::Time::now()).sleep();
      std::cerr << "Init time: " << approx_init_time << " Current time: " << ros::Time::now()
        << endl; };
  EXPECT_TRUE(hb_monitor.healthy())
    << "Heartbeat should be healthy before the end of the initialization period.";
  EXPECT_GE(1, hb_monitor.getNumConnections())
    << "Monitor should be connected to at least one publisher";
  EXPECT_STREQ(heartbeat_topic.c_str(), hb_monitor.heartbeat_name().c_str());
  EXPECT_STREQ(alarm_name.c_str(), hb_monitor.alarm_name().c_str());
  EXPECT_TRUE(listener.query_cleared()) << "The alarm should start out cleared";

  // Test initialization buffer, alarm will not raise during the first n
  //  seconds after it is constructed
  sleep_until(0.75);
  ros::spinOnce();
  EXPECT_FALSE(listener.query_raised())
    << "Alarm should not be raised before the end of the initialization period.";
  sleep_until(0.85);
  ros::spinOnce();
  EXPECT_TRUE(listener.query_raised())
    << "The initialization period has ended so the alarm should have been raised.";

  // The alarm should be raised once the heartbeat stops publishing

  // The alarm should be cleared if the heartbeat comes back

  return;
}
