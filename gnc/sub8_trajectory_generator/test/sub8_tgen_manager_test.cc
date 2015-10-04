/**
 * Author: Patrick Emami
 * Date: 9/30/15
 */
#include <gtest/gtest.h>
#include "sub8_tgen_manager.h"
#include "sub8_state_space.h"

using sub8::trajectory_generator::Sub8TGenManager;
using sub8::trajectory_generator::Sub8StateSpace;

TEST(Sub8TGenManager, testConstructorRRT) {
  boost::shared_ptr<Sub8TGenManager> test_tgen_manager(new Sub8TGenManager(2));
}

TEST(Sub8TGenManager, testConstructorPDST) {
  boost::shared_ptr<Sub8TGenManager> test_tgen_manager(new Sub8TGenManager(1));
}

TEST(Sub8TGenManager, testSetProblemDefinition) {
  // Instantiate RRT planner
  boost::shared_ptr<Sub8TGenManager> test_tgen_manager(new Sub8TGenManager(2));
  boost::shared_ptr<Sub8StateSpace> test_state_space(new Sub8StateSpace());

  // Set up start and goal states
  State* start_state = test_state_space->allocState();
  State* goal_state = test_state_space->allocState();

  // TGen Manager handles internal validation of the problem definition
  // after setting it
  test_tgen_manager->setProblemDefinition(start_state, goal_state);
}

// TODO
TEST(Sub8TGenManager, testSolveRRT) { ADD_FAILURE() << "Unimplemented test"; }

// TODO
TEST(Sub8TGenManager, testTrajectoryValidation) {
  ADD_FAILURE() << "Unimplemented test";
}