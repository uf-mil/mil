#include <ros_alarms/listener.hpp>
#include <ros_alarms/broadcaster.hpp>
#include <iostream>
#include <functional>
#include <gtest/gtest.h>

TEST(BroadcasterTest, broadcasterTest)
{
  ros::NodeHandle nh;
  string alarm_name = "test_alarm";
  AlarmProxy pxy {"test_alarm", false, "test_alarm_client_node", "", "json", 5};

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
