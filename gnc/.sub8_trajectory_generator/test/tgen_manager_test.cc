/**
 * Author: Patrick Emami
 * Date: 9/30/15
 */
#include "tgen_manager.h"
#include "tgen_common.h"
#include "sub8_msgs/Path.h"
#include "sub8_state_space.h"
#include "ompl/base/StateSampler.h"
#include <sub8_alarm/alarm_helpers.h>
#include <sub8_msgs/PathPoint.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <boost/math/constants/constants.hpp>
#include <ros/console.h>

using sub8::trajectory_generator::TGenManager;
using sub8::trajectory_generator::TGenManagerPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using sub8::AlarmBroadcasterPtr;
using sub8::AlarmBroadcaster;
using ompl::base::StateSamplerPtr;

class TGenManagerTest : public ::testing::Test {
 public:
  TGenManagerTest()
      : _node_handle(new ros::NodeHandle()),
        _test_space(new Sub8StateSpace()){};

  boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return _node_handle; }

  TGenManagerPtr tgenManagerFactory() {
    AlarmBroadcasterPtr ab(new AlarmBroadcaster(getNodeHandle()));
    return TGenManagerPtr(new TGenManager(std::move(ab)));
  }

  // return a reference to the state space
  const ompl::base::StateSpacePtr& ss() const { return _test_space; }

  // Sample a random start and goal state
  std::vector<State*> randomProblemDef() {
    std::vector<State*> pd;

    // Set up start and goal states
    State* start_state = ss()->allocState();
    State* goal_state = ss()->allocState();

    StateSamplerPtr state_sampler = ss()->allocStateSampler();
    state_sampler->sampleUniform(start_state);
    state_sampler->sampleUniform(goal_state);

    pd.push_back(start_state);
    pd.push_back(goal_state);

    return pd;
  }

 private:
  boost::shared_ptr<ros::NodeHandle> _node_handle;
  ompl::base::StateSpacePtr _test_space;
};

TEST_F(TGenManagerTest, testSetProblemDefinition) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  std::vector<State*> pd = randomProblemDef();
  test_tgen_manager->setProblemDefinition(pd[0], pd[1]);

  ss()->freeState(pd[0]);
  ss()->freeState(pd[1]);
}

TEST_F(TGenManagerTest, testBadProblemDefinition) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  // Set up start and goal states
  State* start_state = ss()->allocState();
  State* goal_state = ss()->allocState();

  std::vector<State*> pd = randomProblemDef();
  pd[0]->as<Sub8StateSpace::StateType>()->setX(10000);

  ASSERT_FALSE(test_tgen_manager->setProblemDefinition(pd[0], pd[1]));
}

TEST_F(TGenManagerTest, testGeneratePath) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  std::vector<State*> pd = randomProblemDef();
  test_tgen_manager->setProblemDefinition(pd[0], pd[1]);

  ROS_WARN("Testing TGEN planning with a random start and goal...");

  ASSERT_TRUE(test_tgen_manager->solve());

  ss()->freeState(pd[0]);
  ss()->freeState(pd[1]);
}

TEST_F(TGenManagerTest, testGetPath) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  std::vector<State*> pd = randomProblemDef();
  test_tgen_manager->setProblemDefinition(pd[0], pd[1]);

  ASSERT_TRUE(test_tgen_manager->solve());

  std::vector<State*> path = test_tgen_manager->getPath();

  ASSERT_GT(path.size(), 2);
}

TEST_F(TGenManagerTest, testGeneratePathMessage) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  std::vector<State*> pd = randomProblemDef();
  test_tgen_manager->setProblemDefinition(pd[0], pd[1]);

  ASSERT_TRUE(test_tgen_manager->solve());

  sub8_msgs::Path path_msg = test_tgen_manager->generatePathMessage();

  std::vector<sub8_msgs::PathPoint> pp = path_msg.path;

  ASSERT_NEAR(pd[0]->as<Sub8StateSpace::StateType>()->getX(), pp[0].position.x,
              0.01);
  ASSERT_NEAR(pd[0]->as<Sub8StateSpace::StateType>()->getY(), pp[0].position.y,
              0.01);
  ASSERT_NEAR(pd[0]->as<Sub8StateSpace::StateType>()->getZ(), pp[0].position.z,
              0.01);
  ASSERT_NEAR(pd[0]->as<Sub8StateSpace::StateType>()->getYaw(), pp[0].yaw,
              0.01);
}

// TODO - needs Octomap
TEST_F(TGenManagerTest, testPathValidation) {
  ADD_FAILURE() << "Unimplemented test";
}

TEST_F(TGenManagerTest, testStateToPathPoint) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  State* state = ss()->allocState();
  state->as<Sub8StateSpace::StateType>()->setXYZ(1, 1, 1);

  state->as<Sub8StateSpace::StateType>()->setYaw(
      boost::math::constants::pi<double>());

  sub8_msgs::PathPoint path_point = test_tgen_manager->stateToPathPoint(state);

  ASSERT_NEAR(state->as<Sub8StateSpace::StateType>()->getX(),
              path_point.position.x, 0.01);
  ASSERT_NEAR(state->as<Sub8StateSpace::StateType>()->getY(),
              path_point.position.y, 0.01);
  ASSERT_NEAR(state->as<Sub8StateSpace::StateType>()->getZ(),
              path_point.position.z, 0.01);
  ASSERT_NEAR(state->as<Sub8StateSpace::StateType>()->getYaw(), path_point.yaw,
              0.01);

  ss()->freeState(state);
}

TEST_F(TGenManagerTest, testPoseToState) {
  TGenManagerPtr test_tgen_manager = tgenManagerFactory();

  State* state = ss()->allocState();

  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 1;
  pose.position.z = 2;
  pose.orientation.x = 1;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  state = test_tgen_manager->poseToState(pose);

  ASSERT_NEAR(pose.position.x, state->as<Sub8StateSpace::StateType>()->getX(),
              0.01);
  ASSERT_NEAR(pose.position.y, state->as<Sub8StateSpace::StateType>()->getY(),
              0.01);
  ASSERT_NEAR(pose.position.z, state->as<Sub8StateSpace::StateType>()->getZ(),
              0.01);
  ASSERT_NEAR(0, state->as<Sub8StateSpace::StateType>()->getYaw(), 0.01);

  ss()->freeState(state);
}