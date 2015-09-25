/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "sub8_space_information.h"
#include "sub8_state_validity_checker.h"

using sub8::trajectory_generator::Sub8SpaceInformation;
using sub8::trajectory_generator::Sub8SpaceInformationGenerator; 
using sub8::trajectory_generator::Sub8SpaceInformationPtr;
using sub8::trajectory_generator::Sub8StateValidityChecker; 
using sub8::trajectory_generator::Sub8StateValidityCheckerPtr;

Sub8SpaceInformationPtr Sub8SpaceInformationGenerator::generate() {
	StateSpacePtr space(new Sub8StateSpace());
	// Set bounds for Sub8StateSpace
	setStateSpaceBounds(space);

	ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, _CSPACE_DIMS));
	// Set bounds for cspace
	setControlSpaceBounds(cspace);
	
	Sub8SpaceInformationPtr si_ptr(new Sub8SpaceInformation(space, cspace));
	
	// Set state validity checker
	Sub8StateValidityCheckerPtr vc_ptr(new Sub8StateValidityChecker(si_ptr));
	si_ptr->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(vc_ptr));

	// Must be run once before use
	si_ptr->setup();

	return si_ptr;
}

void Sub8SpaceInformationGenerator::setStateSpaceBounds(const StateSpacePtr& space) {
	// TODO 
} 

void Sub8SpaceInformationGenerator::setControlSpaceBounds(const ControlSpacePtr& space) {
	// TODO
}

void Sub8SpaceInformation::propagate(const State* state, const Control* control, const double duration, State* result) const {
	// TODO
	return; 
}