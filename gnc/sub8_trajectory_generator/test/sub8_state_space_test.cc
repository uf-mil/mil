/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "sub8_state_space.h"
#include <cmath>

using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::Sub8StateSpacePtr; 

// Test constructor and run sanity checks
TEST(Sub8StateSpaceTest, sanityCheckTest) {
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());
  test_state_space->sanityChecks();
}

// Test to ensure subspaces were correctly constructed
TEST(Sub8StateSpaceTest, dimensionTest) {
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());
  ASSERT_EQ(test_state_space->getDimension(), 13)
      << "Sub8's state space should have 13 dimensions";
}

// Test setting bounds on a subspace
TEST(Sub8StateSpaceTest, boundsTest) {
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());

  // Bounds on position (x, y, z)
  RealVectorBounds bounds(3);
  bounds.setLow(0, -1);   // x low
  bounds.setHigh(0, 1);   // x high
  bounds.setLow(1, -5);   // y low
  bounds.setHigh(1, 5);   // y high
  bounds.setLow(2, -10);  // z low
  bounds.setHigh(2, 10);  // z high

  test_state_space->set_volume_bounds(bounds);

  RealVectorBounds r = test_state_space->get_volume_bounds();

  // Compare volumes
  EXPECT_EQ(bounds.getVolume(), r.getVolume());
}

TEST(Sub8StateSpaceTest, euclideanDistanceTest) {
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());

  State* state1 = test_state_space->allocState();
  state1->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);

  State* state2 = test_state_space->allocState();
  state2->as<Sub8StateSpace::StateType>()->setPosition(-1, -1, -1);

  ASSERT_EQ(sqrt(12), test_state_space->distance(state1, state2));
}

TEST(Sub8StateSpaceTest, equalStatesTest) {
  ADD_FAILURE() << "Unimplemented test";
}

TEST(Sub8StateSpace, maxExtentTest) { ADD_FAILURE() << "Unimplemented test"; }
