/**
 * Author: Patrick Emami
 * Date: 9/30/15
 */
#include <gtest/gtest.h>
#include "tgen_manager.h"
#include "tgen_common.h"
#include "sub8_state_space.h"
#include "sub8_msgs/Waypoint.h"
#include "sub8_msgs/Trajectory.h"
#include <sub8_alarm/alarm_helpers.h>
#include "tgen_thruster_info.h"

using sub8::trajectory_generator::TGenManager;
using sub8::trajectory_generator::TGenManagerPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::Sub8StateSpacePtr;
using sub8::trajectory_generator::TGenThrusterInfo;
using sub8::trajectory_generator::TGenThrusterInfoPtr;
using sub8::trajectory_generator::Matrix2_8d;
using sub8::trajectory_generator::_CSPACE_DIMS;
using sub8::AlarmBroadcasterPtr;
using sub8::AlarmBroadcaster;

class TGenManagerTest : public ::testing::Test {
 public:
  TGenManagerTest() : _node_handle(new ros::NodeHandle()){};
  boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return _node_handle; }

  TGenManagerPtr tgenManagerFactory(int planner_id) {
    AlarmBroadcasterPtr ab(new AlarmBroadcaster(getNodeHandle()));
    TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());
    Matrix2_8d test_thruster_ranges;
    test_thruster_ranges.row(0) = -MatrixXd::Ones(1, _CSPACE_DIMS);
    test_thruster_ranges.row(1) = MatrixXd::Ones(1, _CSPACE_DIMS);
    return TGenManagerPtr(
        new TGenManager(planner_id, test_thruster_ranges, thruster_info, ab));
  }

 private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
};

TEST_F(TGenManagerTest, testConstructorRRT) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory(2);
}

TEST_F(TGenManagerTest, testConstructorPDST) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory(1);
}

TEST_F(TGenManagerTest, testSetProblemDefinition) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory(2);
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());

  // Set up start and goal states
  State* start_state = test_state_space->allocState();
  State* goal_state = test_state_space->allocState();

  // TGen Manager handles internal validation of the problem definition
  // after setting it
  test_tgen_manager->setProblemDefinition(start_state, goal_state);

  test_state_space->freeState(start_state);
  test_state_space->freeState(goal_state);
}

// TODO
TEST_F(TGenManagerTest, testSolveRRT) { ADD_FAILURE() << "Unimplemented test"; }

// TODO
TEST_F(TGenManagerTest, testTrajectoryValidation) {
  ADD_FAILURE() << "Unimplemented test";
}

TEST_F(TGenManagerTest, testStateToWaypoint) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory(2);
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());

  State* state = test_state_space->allocState();
  state->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setAngularVelocity(1, 1, 1);
  state->as<Sub8StateSpace::StateType>()->setOrientation(0.78, 0, 0, 0.78);

  sub8_msgs::Waypoint wpoint = test_tgen_manager->stateToWaypoint(state);

  Vector3d pos;
  state->as<Sub8StateSpace::StateType>()->getPosition(pos);

  ASSERT_EQ(pos(0), wpoint.pose.position.x);
  ASSERT_EQ(pos(1), wpoint.pose.position.y);
  ASSERT_EQ(pos(2), wpoint.pose.position.z);

  test_state_space->freeState(state);
}

TEST_F(TGenManagerTest, testWaypointToState) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory(2);
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());
  State* state = test_state_space->allocState();

  boost::shared_ptr<sub8_msgs::Waypoint> wpoint(new sub8_msgs::Waypoint());
  wpoint->pose.position.x = 1;
  wpoint->pose.position.y = 1;
  wpoint->pose.position.z = 2;

  state = test_tgen_manager->waypointToState(wpoint);

  Vector3d pos;
  state->as<Sub8StateSpace::StateType>()->getPosition(pos);

  ASSERT_EQ(wpoint->pose.position.x, pos(0));
  ASSERT_EQ(wpoint->pose.position.y, pos(1));
  ASSERT_EQ(wpoint->pose.position.z, pos(2));

  test_state_space->freeState(state);
}

TEST_F(TGenManagerTest, testGetTrajectory) {
  ADD_FAILURE() << "Unimplemented test";
}
