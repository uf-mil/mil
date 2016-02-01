/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "space_information_generator.h"

using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using ompl::base::SpaceInformationPtr;

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
  SpaceInformationPtr si = ss_gen->generate();

  ASSERT_EQ(si->getStateDimension(), 4)
      << "The Sub8 state space should have 4 dimensions";

  // Verify that the SpaceInformation object's setup method has been called
  ASSERT_TRUE(si->isSetup());
}

TEST_F(SpaceInformationTest, testSetStateSpaceBounds) {
    ADD_FAILURE() << "Unimplemented test";
}