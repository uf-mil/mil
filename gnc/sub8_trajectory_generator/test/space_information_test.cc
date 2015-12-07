/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "space_information_generator.h"
#include "ompl/control/SpaceInformation.h"
#include "sub_dynamics.h"
#include "tgen_thruster_info.h"

using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::SubDynamics;
using sub8::trajectory_generator::SubDynamicsPtr;
using sub8::trajectory_generator::TGenThrusterInfo;
using sub8::trajectory_generator::TGenThrusterInfoPtr;
using sub8::trajectory_generator::Matrix2_8d;
using sub8::trajectory_generator::_CSPACE_DIMS;
using ompl::control::SpaceInformationPtr;

class SpaceInformationTest : public ::testing::Test {
 public:
  SpaceInformationTest() : _node_handle(new ros::NodeHandle()){};
  boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return _node_handle; }

  SpaceInformationGeneratorPtr spaceInformationFactory() {
    return SpaceInformationGeneratorPtr(new SpaceInformationGenerator());
  }

 private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
};

// Test the "generate" factory method, and verify that it correctly initializes
// the Sub8SpaceInformation object
TEST_F(SpaceInformationTest, testSpaceInformationGenerator) {
  SpaceInformationGeneratorPtr ss_gen = spaceInformationFactory();
  TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());
  SubDynamicsPtr sub_dynamics(new SubDynamics(thruster_info));

  Matrix2_8d test_thruster_ranges;
  test_thruster_ranges.row(0) = -MatrixXd::Ones(1, _CSPACE_DIMS);
  test_thruster_ranges.row(1) = MatrixXd::Ones(1, _CSPACE_DIMS);

  SpaceInformationPtr si = ss_gen->generate(sub_dynamics, test_thruster_ranges);

  ASSERT_EQ(si->getStateDimension(), 13)
      << "The Sub8 state space should have 13 dimensions";

  // Verify that the SpaceInformation object's setup method has been called
  ASSERT_TRUE(si->isSetup());
}

TEST_F(SpaceInformationTest, testParameters) {
  double checking_res;
  double prop_step_size;
  double min_control_duration;
  double max_control_duration;
  EXPECT_TRUE(ros::param::get("state_validity_checking_resolution", checking_res));
  EXPECT_TRUE(ros::param::get("propagation_step_size", prop_step_size));
  EXPECT_TRUE(ros::param::get("min_control_duration", min_control_duration));
  EXPECT_TRUE(ros::param::get("max_control_duration", max_control_duration));
}