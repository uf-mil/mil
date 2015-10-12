/**
 * Author: Patrick Emami
 * Date: 9/30/15
 */
#include <gtest/gtest.h>
#include "sub8_tgen_manager.h"
#include "sub8_state_space.h"
#include "sub8_msgs/Waypoint.h"
#include "sub8_msgs/Trajectory.h"

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

TEST(Sub8TGenManager, testStateToWaypoint) {
  boost::shared_ptr<Sub8TGenManager> test_tgen_manager(new Sub8TGenManager(2));
  boost::shared_ptr<Sub8StateSpace> test_state_space(new Sub8StateSpace());

  State* state = test_state_space->allocState();
  state->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setAngularVelocity(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setOrientation(0.78, 0, 0, 0.78);

  sub8_msgs::Waypoint wpoint = test_tgen_manager->stateToWaypoint(state);

  std::vector<double> pos;
  state->as<Sub8StateSpace::StateType>()->getPosition(pos); 

  ASSERT_EQ(pos[0], wpoint.pose.position.x);
  ASSERT_EQ(pos[1], wpoint.pose.position.y);
  ASSERT_EQ(pos[2], wpoint.pose.position.z); 
}

TEST(Sub8TGenManager, testWaypointToState) {
  boost::shared_ptr<Sub8TGenManager> test_tgen_manager(new Sub8TGenManager(2));
  boost::shared_ptr<Sub8StateSpace> test_state_space(new Sub8StateSpace());
  State* state = test_state_space->allocState();

  boost::shared_ptr<sub8_msgs::Waypoint> wpoint(new sub8_msgs::Waypoint()); 
  wpoint->pose.position.x = 1; 
  wpoint->pose.position.y = 1; 
  wpoint->pose.position.z = 2; 

  state = test_tgen_manager->waypointToState(wpoint);

  std::vector<double> pos;  
  state->as<Sub8StateSpace::StateType>()->getPosition(pos); 

  ASSERT_EQ(wpoint->pose.position.x, pos[0]); 
  ASSERT_EQ(wpoint->pose.position.y, pos[1]); 
  ASSERT_EQ(wpoint->pose.position.z, pos[2]); 
}

TEST(Sub8TGenManager, getTrajectory) {
    ADD_FAILURE() << "Unimplemented test";
}