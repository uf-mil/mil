/**
 * Author: Patrick Emami
 * Date: 11/11/2015
 */

#include "gtest/gtest.h"
#include "sub8_alarm/alarm_helpers.h"
#include <string>

using sub8::AlarmBroadcaster;
using sub8::AlarmBroadcasterPtr;
using sub8::AlarmRaiser;
using sub8::AlarmRaiserPtr;
using sub8::PublisherPtr;

class AlarmHelpersTest : public ::testing::Test {
 public:
  AlarmHelpersTest() : _node_handle(new ros::NodeHandle()){};
  boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return _node_handle; }

 private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
};

TEST_F(AlarmHelpersTest, testAlarmBroadcasterConstructor) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
}

TEST_F(AlarmHelpersTest, testAlarmRaiserConstructor) {
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";

  // For testing, just pass in a null Publisher object
  // It's okay to allow this behavior, since this constructor
  // is never explicitly called
  PublisherPtr pub = nullptr;
  AlarmRaiserPtr alarm_raiser_ptr(new AlarmRaiser(alarm_name, node_name, pub));

  EXPECT_EQ(alarm_name, alarm_raiser_ptr->getAlarmName());
  EXPECT_EQ(node_name, alarm_raiser_ptr->getNodeName());
  EXPECT_EQ(empty, alarm_raiser_ptr->getProblemDescription());
  EXPECT_EQ(false, alarm_raiser_ptr->isActionRequired());
}

TEST_F(AlarmHelpersTest, testAddAlarm) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";

  // Sets action_required to true
  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name, true);

  // Assert members are set
  EXPECT_EQ(alarm_name, new_alarm->getAlarmName());
  EXPECT_EQ(node_name, new_alarm->getNodeName());
  EXPECT_EQ(empty, new_alarm->getProblemDescription());
  EXPECT_EQ(true, new_alarm->isActionRequired());
}

TEST_F(AlarmHelpersTest, testRaisingAlarm) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";

  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name, true);

  const std::string problem_description = "Entering black hole";
  const std::string parameters = "";

  boost::shared_ptr<sub8_msgs::Alarm> alarm_msg =
      new_alarm->raiseAlarm(problem_description, parameters);

  EXPECT_EQ(problem_description, alarm_msg->problem_description);
  EXPECT_EQ(node_name, alarm_msg->node_name);
  EXPECT_EQ(true, alarm_msg->action_required);
  EXPECT_EQ(2, alarm_msg->severity);
}

// Test building and decoding JSON
TEST_F(AlarmHelpersTest, testJSONBlob) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";
  // TODO: Test whether the AlarmHandler actually accepts this
  const std::string json_string =
      "{ \"ShowMe\": [\"What\", \"You\", \"Got\"] }";

  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name, true);

  const std::string problem_description = "Entering black hole";

  // The AlarmHandler node will decode the JSON
  boost::shared_ptr<sub8_msgs::Alarm> alarm_msg =
      new_alarm->raiseAlarm(problem_description, json_string);

  EXPECT_EQ(problem_description, alarm_msg->problem_description);
  EXPECT_EQ(node_name, alarm_msg->node_name);
  EXPECT_EQ(true, alarm_msg->action_required);
  EXPECT_EQ(2, alarm_msg->severity);
  EXPECT_EQ(json_string, alarm_msg->parameters);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "alarm_helpers_test");
  return RUN_ALL_TESTS();
}
