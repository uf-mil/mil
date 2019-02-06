/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "space_information_generator.h"
#include "sub8_state_space.h"

using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using ompl::base::SpaceInformationPtr;
using ompl::base::StateSpacePtr;
using ompl::base::RealVectorBounds;

class SpaceInformationTest : public ::testing::Test
{
public:
  SpaceInformationTest() : _node_handle(new ros::NodeHandle()){};
  boost::shared_ptr<ros::NodeHandle> getNodeHandle()
  {
    return _node_handle;
  }

  SpaceInformationGeneratorPtr spaceInformationFactory()
  {
    return SpaceInformationGeneratorPtr(new SpaceInformationGenerator());
  }

private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
};

// Test the "generate" factory method, and verify that it correctly initializes
// the Sub8SpaceInformation object
TEST_F(SpaceInformationTest, testSpaceInformationGenerator)
{
  SpaceInformationGeneratorPtr ss_gen = spaceInformationFactory();
  SpaceInformationPtr si = ss_gen->generate();

  ASSERT_EQ(si->getStateDimension(), 4) << "The Sub8 state space should have 4 dimensions";

  // Verify that the SpaceInformation object's setup method has been called
  ASSERT_TRUE(si->isSetup());
}

TEST_F(SpaceInformationTest, testSetStateSpaceBounds)
{
  SpaceInformationGeneratorPtr ss_gen = spaceInformationFactory();
  SpaceInformationPtr si = ss_gen->generate();
  StateSpacePtr space = si->getStateSpace();
  // Bounds on position (x, y, z)
  RealVectorBounds pos_bounds(3);

  ros::param::get("xmin", pos_bounds.low[0]);
  ros::param::get("xmax", pos_bounds.high[0]);
  ros::param::get("ymin", pos_bounds.low[1]);
  ros::param::get("ymax", pos_bounds.high[1]);
  ros::param::get("zmin", pos_bounds.low[2]);
  ros::param::get("zmax", pos_bounds.high[2]);

  // set bounds on the R^3 component
  space->as<Sub8StateSpace>()->setBounds(pos_bounds);

  RealVectorBounds check_pos = space->as<Sub8StateSpace>()->getBounds();
  ASSERT_EQ(pos_bounds.low[0], check_pos.low[0]);
  ASSERT_EQ(pos_bounds.high[0], check_pos.high[0]);
  ASSERT_EQ(pos_bounds.low[1], check_pos.low[1]);
  ASSERT_EQ(pos_bounds.high[1], check_pos.high[1]);
  ASSERT_EQ(pos_bounds.low[2], check_pos.low[2]);
  ASSERT_EQ(pos_bounds.high[2], check_pos.high[2]);
}