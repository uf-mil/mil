/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "sub8_state_space.h"
#include <cmath>
#include <limits>

using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::Sub8StateSpacePtr;
using sub8::trajectory_generator::Vector13d;
using ompl::base::StateSamplerPtr;

class Sub8StateSpaceTest : public ::testing::Test {
 public:
  Sub8StateSpaceTest() {
    std::vector<double> weights = {1, 1, 1, 1};
    test_state_space =
        Sub8StateSpacePtr(new Sub8StateSpace(std::move(weights)));
  }
  const Sub8StateSpacePtr& sub8_ss() const { return test_state_space; }

 private:
  Sub8StateSpacePtr test_state_space;
};

// Test constructor and run sanity checks
TEST_F(Sub8StateSpaceTest, sanityCheckTest) { sub8_ss()->sanityChecks(); }

// Test to ensure subspaces were correctly constructed
TEST_F(Sub8StateSpaceTest, dimensionTest) {
  ASSERT_EQ(sub8_ss()->getDimension(), 12)
      << "Sub8's state space should have 12 dimensions";
}

// Test setting bounds on a subspace
TEST_F(Sub8StateSpaceTest, boundsTest) {
  // Bounds on position (x, y, z)
  RealVectorBounds bounds(3);
  bounds.setLow(0, -1);   // x low
  bounds.setHigh(0, 1);   // x high
  bounds.setLow(1, -5);   // y low
  bounds.setHigh(1, 5);   // y high
  bounds.setLow(2, -10);  // z low
  bounds.setHigh(2, 10);  // z high

  sub8_ss()->set_volume_bounds(bounds);
  RealVectorBounds r = sub8_ss()->get_volume_bounds();

  // Compare volumes
  EXPECT_EQ(bounds.getVolume(), r.getVolume());
}

TEST_F(Sub8StateSpaceTest, sampleUniformTest) {
  // Test whether state sampling is functioning properly

  // Bounds on position (x, y, z)
  RealVectorBounds pos_bounds(3);
  // Bounds on velocity (x, y, z)
  RealVectorBounds vel_bounds(3);
  // Bounds on angular velocity (wx, wy, wz)
  RealVectorBounds w_bounds(3);

  ros::param::get("xmin", pos_bounds.low[0]);
  ros::param::get("xmax", pos_bounds.high[0]);
  ros::param::get("ymin", pos_bounds.low[1]);
  ros::param::get("ymax", pos_bounds.high[1]);
  ros::param::get("zmin", pos_bounds.low[2]);
  ros::param::get("zmax", pos_bounds.high[2]);

  ros::param::get("v_xmin", vel_bounds.low[0]);
  ros::param::get("v_xmax", vel_bounds.high[0]);
  ros::param::get("v_ymin", vel_bounds.low[1]);
  ros::param::get("v_ymax", vel_bounds.high[1]);
  ros::param::get("v_zmin", vel_bounds.low[2]);
  ros::param::get("v_zmax", vel_bounds.high[2]);

  ros::param::get("w_xmin", w_bounds.low[0]);
  ros::param::get("w_xmax", w_bounds.high[0]);
  ros::param::get("w_ymin", w_bounds.low[1]);
  ros::param::get("w_ymax", w_bounds.high[1]);
  ros::param::get("w_zmin", w_bounds.low[2]);
  ros::param::get("w_zmax", w_bounds.high[2]);

  sub8_ss()->set_volume_bounds(pos_bounds);
  sub8_ss()->set_linear_velocity_bounds(vel_bounds);
  sub8_ss()->set_angular_velocity_bounds(w_bounds);

  State* a = sub8_ss()->allocState();

  StateSamplerPtr ss = sub8_ss()->allocStateSampler();
  ss->sampleUniform(a);

  Vector13d state_a = a->as<Sub8StateSpace::StateType>()->getState();

  bool all_nonzero = true;
  for (unsigned int i = 0; i < state_a.size(); ++i) {
    if (state_a(i) == 0) {
      all_nonzero = false;
    }
  }

  ASSERT_TRUE(all_nonzero) << state_a;

  sub8_ss()->freeState(a);
}

TEST_F(Sub8StateSpaceTest, distanceBetweenSameOrientationTest) {
  // Take angle between two orientations as their distance
  State* a = sub8_ss()->allocState();

  StateSamplerPtr ss = sub8_ss()->allocStateSampler();
  ss->sampleUniform(a);

  Vector13d state_a = a->as<Sub8StateSpace::StateType>()->getState();

  double lambda = state_a.segment(9, 4).dot(state_a.segment(9, 4));

  // Angle between an orientation and itself should be 0
  ASSERT_NEAR(0, (1 - lambda), std::numeric_limits<float>::epsilon())
      << state_a.segment(9, 4);

  sub8_ss()->freeState(a);
}

TEST_F(Sub8StateSpaceTest, distanceFromStateToItselfTest) {
  State* a = sub8_ss()->allocState();

  StateSamplerPtr ss = sub8_ss()->allocStateSampler();
  a->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 3, 1);
  a->as<Sub8StateSpace::StateType>()->setAngularVelocity(0.21, 0.77, 1.212);
  a->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  double dist = sub8_ss()->distance(a, a);

  ASSERT_NEAR(0, dist, std::numeric_limits<float>::epsilon()) << dist;

  sub8_ss()->freeState(a);
}

TEST_F(Sub8StateSpaceTest, distanceFromZeroTest) {
  State* a = sub8_ss()->allocState();
  a->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setAngularVelocity(0, 0, 0);
  a->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  State* b = sub8_ss()->allocState();
  b->as<Sub8StateSpace::StateType>()->setPosition(0, 0, 0);
  b->as<Sub8StateSpace::StateType>()->setLinearVelocity(0, 0, 0);
  b->as<Sub8StateSpace::StateType>()->setAngularVelocity(0, 0, 0);
  b->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  double dist = sub8_ss()->distance(a, b);

  ASSERT_EQ((sqrt(3) + sqrt(3)), dist);
  
  sub8_ss()->freeState(a);
  sub8_ss()->freeState(b);
}

TEST_F(Sub8StateSpaceTest, distanceTest) {
  State* a = sub8_ss()->allocState();
  a->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setAngularVelocity(0, 0, 0);
  a->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  State* b = sub8_ss()->allocState();
  b->as<Sub8StateSpace::StateType>()->setPosition(5, 1, 1);
  b->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  b->as<Sub8StateSpace::StateType>()->setAngularVelocity(0, 0, 0);
  b->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  double dist1 = sub8_ss()->distance(a, b);

  // verify that the distance is larger between two points when
  // the sub also has to change its orientation, even slightly
  b->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 10);

  double dist2 = sub8_ss()->distance(a, b);

  ASSERT_GT(dist2, dist1);

  sub8_ss()->freeState(a);
  sub8_ss()->freeState(b);
}

TEST_F(Sub8StateSpaceTest, equalStatesTest) {
  State* a = sub8_ss()->allocState();
  a->as<Sub8StateSpace::StateType>()->setPosition(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setLinearVelocity(1, 1, 1);
  a->as<Sub8StateSpace::StateType>()->setAngularVelocity(0, 0, 0);
  a->as<Sub8StateSpace::StateType>()->setOrientation(1, 0, 0, 0);

  ASSERT_TRUE(sub8_ss()->equalStates(a, a));

  sub8_ss()->freeState(a);
}