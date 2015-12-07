/**
 * Author: Patrick Emami
 * Date: 10/27/15
 */
#include <gtest/gtest.h>
#include "sub8_state_space.h"
#include "sub_dynamics.h"
#include "tgen_thruster_info.h"
#include "tgen_common.h"
#include "ompl/control/ODESolver.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include <cmath>

namespace oc = ompl::control;

using sub8::trajectory_generator::SubDynamicsPtr;
using sub8::trajectory_generator::SubDynamics;
using sub8::trajectory_generator::TGenThrusterInfo;
using sub8::trajectory_generator::TGenThrusterInfoPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::Sub8StateSpacePtr;

using sub8::trajectory_generator::Matrix3_8d;
using sub8::trajectory_generator::_CSPACE_DIMS;
using sub8::trajectory_generator::Vector13d;

using ompl::control::Control;
using ompl::control::ControlSpacePtr;

TEST(SubDynamicsTest, testSubDynamicsConstructor) {
  TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());
  SubDynamicsPtr sub_dynamics(new SubDynamics(thruster_info));
}

TEST(SubDynamicsTest, testVector13dConstructor) {
  std::vector<double> test;
  for (int i = 0; i < 13; ++i) test.push_back(i);
  Vector13d test_vect(test.data());
  ASSERT_EQ(13, test_vect.size());
}

TEST(SubDynamicsTest, testSegmentAssignment) {
  Vector13d v1;
  Vector13d v2;
  for (int i = 0; i < 13; ++i) v1(i) = i;
  for (int j = 0; j < 13; ++j) v2(j) = j * 2;

  v2.segment(0, 3) = v1.segment(3, 3);

  ASSERT_EQ(v1(3), v2(0));
  ASSERT_EQ(v1(4), v2(1));
  ASSERT_EQ(v1(5), v2(2));
}

// Use a contrived example to test the functionality
// The test values were obtained through
// a MATLAB implementation of the dynamics equations
//
// Will need to verify the correctness further
// with integration tests
TEST(SubDynamicsTest, testSubDynamicsODE) {
  Sub8StateSpacePtr test_state_space(new Sub8StateSpace());
  TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());

  Vector3d FLV_p, FLL_p, FRV_p, FRL_p, BLV_p, BLL_p, BRV_p, BRL_p;
  Vector3d FLV_d, FLL_d, FRV_d, FRL_d, BLV_d, BLL_d, BRV_d, BRL_d;

  FLV_p << 0.1583, 0.169, 0.0142;
  FLL_p << 0.2678, 0.2795, 0;
  FRV_p << 0.1583, -0.169, 0.0142;
  FRL_p << 0.2678, -0.2795, 0;
  BLV_p << -0.1583, 0.169, 0.0142;
  BLL_p << -0.2678, 0.2795, 0;
  BRV_p << -0.1583, -0.169, 0.0142;
  BRL_p << -0.2678, -0.2795, 0;

  Matrix3_8d L;
  L << FLV_p, FLL_p, FRV_p, FRL_p, BLV_p, BLL_p, BRV_p, BRL_p;

  // Set the lever arms matrix
  thruster_info->L = L;

  FLV_d << 0, 0, -1;
  FLL_d << -0.866, 0.5, 0;
  FRV_d << 0, 0, -1;
  FRL_d << -0.866, -0.5, 0;
  BLV_d << 0, 0, 1;
  BLL_d << 0.866, 0.5, 0;
  BRV_d << 0, 0, 1;
  BRL_d << 0.866, -0.5, 0;

  Matrix3_8d D;
  D << FLV_d, FLL_d, FRV_d, FRL_d, BLV_d, BLL_d, BRV_d, BRL_d;

  // Set the thruster directions matrix
  thruster_info->D = D;

  SubDynamicsPtr sub_dynamics(new SubDynamics(thruster_info));

  oc::ODESolver::StateType q;
  oc::ODESolver::StateType qdot;
  q.push_back(1);  // start at (1,1,1)
  q.push_back(1);
  q.push_back(1);
  q.push_back(1);  // 1 m/s in x direction
  q.push_back(0);
  q.push_back(0);
  q.push_back(M_PI / 4);  // some angular velocity in x
  q.push_back(0);
  q.push_back(0);
  q.push_back(0.7071);  // orientation in euler angles is [pi/2 0 0], 'xyz'
  q.push_back(0);
  q.push_back(0);
  q.push_back(0.7071);

  ControlSpacePtr cspace(
      new oc::RealVectorControlSpace(test_state_space, _CSPACE_DIMS));
  Control* c = cspace->allocControl();

  // Fire the thrusters in random directions :D
  double* u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
  u[0] = 1;
  u[1] = 1;
  u[2] = 0;
  u[3] = 1;
  u[4] = 1;
  u[5] = 1;
  u[6] = 1;
  u[7] = 0;

  sub_dynamics->ode(q, c, qdot);

  EXPECT_NEAR(1.0, qdot[0], 0.1);
  EXPECT_NEAR(0, qdot[1], 0.1);
  EXPECT_NEAR(0, qdot[2], 0.1);
  EXPECT_NEAR(-0.0346, qdot[3], 0.1);
  EXPECT_NEAR(-0.04, qdot[4], 0.1);
  EXPECT_NEAR(0.02, qdot[5], 0.1);
  EXPECT_NEAR(0.1095, qdot[6], 0.1);
  EXPECT_NEAR(0.4485, qdot[7], 0.1);
  EXPECT_NEAR(0.0426, qdot[8], 0.1);
  EXPECT_NEAR(0.2777, qdot[9], 0.1);
  EXPECT_NEAR(0, qdot[10], 0.1);
  EXPECT_NEAR(0, qdot[11], 0.1);
  EXPECT_NEAR(-0.2777, qdot[12], 0.1);
}
