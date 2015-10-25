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
using ompl::control::SpaceInformationPtr; 

// Test the factory method, and verify that it correctly initializes the
// Sub8SpaceInformation object
TEST(SpaceInformationTest, testSpaceInformationGenerator) {
  // Create a Sub8SpaceInformationPtr using the SpaceInformationGenerator
  SpaceInformationGeneratorPtr ss_gen(new SpaceInformationGenerator());
  TGenThrusterInfoPtr thruster_info(new TGenThrusterInfo());
  SubDynamicsPtr sub_dynamics(new SubDynamics(thruster_info));
  SpaceInformationPtr si = ss_gen->generate(sub_dynamics);

  ASSERT_EQ(si->getStateDimension(), 13)
      << "The Sub8 state space should have 13 dimensions";

  // Verify that the SpaceInformation object's setup method has been called
  ASSERT_TRUE(si->isSetup());
}