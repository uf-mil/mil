/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "sub8_space_information.h"
#include "sub8_state_validity_checker.h"
#include "sub8_ode_solver.h"
#include "ompl/control/ODESolver.h"

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
	
	// Create and set the state validity checker
	Sub8StateValidityCheckerPtr vc_ptr(new Sub8StateValidityChecker(si_ptr));
	si_ptr->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr>(vc_ptr));
	si_ptr->setStateValidityCheckingResolution(0.03); // 3 % -- TODO

 	// Create and set the ODEBasicSolver
 	ODESolverPtr ode_solver(new ompl::control::ODEBasicSolver<>(si_ptr, &sub8ODE));
 	si_ptr->setODESolver(ode_solver);

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

void Sub8SpaceInformation::setODESolver(const ODESolverPtr& ode_solver) {
	_sub8_ode_solver = ode_solver;
}

void Sub8SpaceInformation::propagate(const State* state, const Control* control, const double duration, State* result) const {
	// TODO
	return; 
}