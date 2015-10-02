/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */

#include "sub8_state_space.h"

using sub8::trajectory_generator::Sub8StateSpace; 

ompl::base::State* Sub8StateSpace::allocState() const {
	StateType *state = new StateType();
	allocStateComponents(state);
	return state; 
}

void Sub8StateSpace::freeState(State* state) const {
	CompoundStateSpace::freeState(state);
}

double Sub8StateSpace::distance(const State* state1, const State* state2) const {
	// TODO 
	// return LQR cost to go to state 2 from state 1
	return 0.0;
}

double Sub8StateSpace::getMaximumExtent() const {
	// TODO
	return std::numeric_limits<double>::max();
}

bool Sub8StateSpace::equalStates(const State* state1, const State* state2) const {
	// TODO
	// Do we want to define some error-vector 
	// s.t. State1 == State2 if State1 is within += error-vector of State2? 
	return true;
}