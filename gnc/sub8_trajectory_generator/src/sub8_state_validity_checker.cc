/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "sub8_state_validity_checker.h"

using sub8::trajectory_generator::Sub8StateValidityChecker; 
 
bool Sub8StateValidityChecker::isValid(const State* state) const {
	// TODO
	// Use Octree for collision checking 
	return true;
} 

bool Sub8StateValidityChecker::isValid(const State* state, double& dist) const {
	// TODO
	return true;
}
 
double Sub8StateValidityChecker::clearance(const State* state) const {
	// TODO
	return 0.0;
}