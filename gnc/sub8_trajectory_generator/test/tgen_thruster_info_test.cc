/**
 * Author: Patrick Emami
 * Date: 10/25/15
 */
#include <gtest/gtest.h>
#include "tgen_thruster_info.h"
#include <sub8_msgs/BMatrix.h>
#include <sub8_msgs/ThrusterInfo.h>
#include <ros/ros.h>

using sub8::trajectory_generator::TGenThrusterInfo;
using sub8::trajectory_generator::TGenThrusterInfoPtr;

class TGenThrusterInfoTest : public ::testing::Test {
 public:
  TGenThrusterInfoTest() : _node_handle(new ros::NodeHandle()){};
  boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return _node_handle; }

 private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
};

TEST_F(TGenThrusterInfoTest, testBMatrix) {
  TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());
  std::string b_matrix_service = "b_matrix";
  sub8_msgs::BMatrix B;

  ros::ServiceClient sc =
      getNodeHandle()->serviceClient<sub8_msgs::BMatrix>(b_matrix_service);

  ASSERT_TRUE(sc.exists());

  ASSERT_TRUE(sc.call(B));

  // Grab B matrix;
  thruster_info->init(B.response.B);
}
