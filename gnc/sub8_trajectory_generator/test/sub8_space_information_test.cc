/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#include <gtest/gtest.h>
#include "sub8_space_information.h"

using sub8::trajectory_generator::Sub8SpaceInformation; 
using sub8::trajectory_generator::Sub8SpaceInformationPtr; 
using sub8::trajectory_generator::Sub8SpaceInformationGenerator;
using sub8::trajectory_generator::Sub8SpaceInformationGeneratorPtr;

// Test the factory method, and verify that it correctly initializes the 
// Sub8SpaceInformation object
TEST(Sub8SpaceInformationTest, testSpaceInformationGenerator) {

	// Create a Sub8SpaceInformationPtr using the Sub8SpaceInformationGenerator
	Sub8SpaceInformationGeneratorPtr ss_gen(new Sub8SpaceInformationGenerator()); 
	Sub8SpaceInformationPtr si = ss_gen->generate(); 

	ASSERT_EQ(si->getStateDimension(), 12) << "The Sub8 state space should have 12 dimensions"; 

	// Verify that the SpaceInformation object's setup method has been called
	ASSERT_TRUE(si->isSetup());
}