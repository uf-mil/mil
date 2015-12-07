/**
 * Author: Patrick Emami
 * Date: 11/11/2015
 */

#include "gtest/gtest.h"
#include "sub8_alarm/alarm_helpers.h"
#include <string>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ros/package.h>

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
  EXPECT_TRUE(alarm_raiser_ptr->isActionRequired());
}

TEST_F(AlarmHelpersTest, testAddAlarm) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";

  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name);

  // Assert members are set
  EXPECT_EQ(alarm_name, new_alarm->getAlarmName());
  EXPECT_EQ(node_name, new_alarm->getNodeName());
  EXPECT_EQ(empty, new_alarm->getProblemDescription());
  EXPECT_TRUE(new_alarm->isActionRequired());
}

TEST_F(AlarmHelpersTest, testRaisingAlarm) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  const std::string alarm_name = "test_alarm";
  const std::string node_name = ros::this_node::getName();
  const std::string empty = "";

  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name);

  const std::string problem_description = "Entering black hole";
  const std::string parameters = "";

  boost::shared_ptr<sub8_msgs::Alarm> alarm_msg =
      new_alarm->raiseAlarm(problem_description, parameters);

  EXPECT_EQ(problem_description, alarm_msg->problem_description);
  EXPECT_EQ(node_name, alarm_msg->node_name);
  EXPECT_TRUE(alarm_msg->action_required);
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

  AlarmRaiserPtr new_alarm = alarm_ptr->addAlarm(alarm_name);

  const std::string problem_description = "Entering black hole";

  // The AlarmHandler node will decode the JSON
  boost::shared_ptr<sub8_msgs::Alarm> alarm_msg =
      new_alarm->raiseAlarm(problem_description, json_string);

  EXPECT_EQ(problem_description, alarm_msg->problem_description);
  EXPECT_EQ(node_name, alarm_msg->node_name);
  EXPECT_TRUE(alarm_msg->action_required);
  EXPECT_EQ(2, alarm_msg->severity);
  EXPECT_EQ(json_string, alarm_msg->parameters);
}

TEST_F(AlarmHelpersTest, testBoostFilesystem) {
  namespace fs = ::boost::filesystem;

  char file_sep = '/';
#ifdef _WIN32
  file_sep = "\\";
#endif
  std::vector<std::string> test_alarms;
  std::string pkg_path = ros::package::getPath("sub8_alarm");
  // Will break if our file structure changes :/
  fs::path dirname(pkg_path + file_sep + "test_alarms" + file_sep + "cpp" +
                   file_sep + "cfg");

  ASSERT_TRUE(fs::exists(dirname)) << "\ndirname: " << dirname;
  ASSERT_TRUE(fs::is_directory(dirname)) << dirname;

  fs::recursive_directory_iterator it(dirname);
  fs::recursive_directory_iterator endit;

  // iterate all files in the directory and store all with the extension in
  // a vector for further processing
  while (it != endit) {
    if (fs::is_regular_file(*it)) {
      // Store the dirname + filename
      test_alarms.push_back((dirname.string()) +
                            (it->path().filename().string()));
      ++it;
    }
  }

  ASSERT_FALSE(test_alarms.empty());
}

TEST_F(AlarmHelpersTest, testBoostJSONParser) {
  namespace fs = ::boost::filesystem;

  char file_sep = '/';
#ifdef _WIN32
  file_sep = "\\";
#endif
  std::vector<std::string> test_alarms;
  std::string pkg_path = ros::package::getPath("sub8_alarm");
  // Will break if our file structure changes :/
  fs::path dirname(pkg_path + file_sep + "test_alarms" + file_sep + "cpp" +
                   file_sep + "cfg");
  fs::recursive_directory_iterator it(dirname);

  ASSERT_TRUE(fs::is_regular_file(*it));

  std::string test_alarm = it->path().string();

  std::string alarm_name = "";
  if (test_alarm.find("1") != std::string::npos) {
    alarm_name = "test_alarm_1";
  } else {
    alarm_name = "test_alarm_2";
  }

  boost::property_tree::ptree pt;

  try {
    boost::property_tree::read_json(test_alarm, pt);

    EXPECT_EQ(alarm_name, pt.get<std::string>("alarm_name"));
    EXPECT_TRUE(pt.get<bool>("action_required"));
    EXPECT_EQ(0, pt.get<int>("severity"));
    EXPECT_EQ("This is " + alarm_name,
              pt.get<std::string>("problem_description"));
  } catch (const boost::property_tree::ptree_error& e) {
    EXPECT_EQ(1, 2) << e.what() << "\nfilename: " << test_alarm;
  }
}

TEST_F(AlarmHelpersTest, testAddAlarms) {
  AlarmBroadcasterPtr alarm_ptr(new AlarmBroadcaster(getNodeHandle()));
  std::vector<AlarmRaiserPtr> test_alarms;

  namespace fs = ::boost::filesystem;

  char file_sep = '/';
#ifdef _WIN32
  file_sep = "\\";
#endif
  std::string pkg_path = ros::package::getPath("sub8_alarm");
  // Will break if our file structure changes :/
  fs::path dirname(pkg_path + file_sep + "test_alarms" + file_sep + "cpp" +
                   file_sep + "cfg");
  bool result = alarm_ptr->addAlarms(dirname, test_alarms);

  std::string alarm_name_1 = "test_alarm_1";
  std::string alarm_name_2 = "test_alarm_2";

  if (test_alarms[0]->getAlarmName().find("1") == std::string::npos) {
    alarm_name_1 = "test_alarm_2";
    alarm_name_2 = "test_alarm_1";
  }

  ASSERT_TRUE(result);
  EXPECT_EQ(alarm_name_1, test_alarms[0]->getAlarmName());
  EXPECT_EQ(alarm_name_2, test_alarms[1]->getAlarmName());
  EXPECT_EQ("This is " + alarm_name_1, test_alarms[0]->getProblemDescription());
  EXPECT_EQ("This is " + alarm_name_2, test_alarms[1]->getProblemDescription());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "alarm_helpers_test");
  return RUN_ALL_TESTS();
}
