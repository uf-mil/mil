/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */

#include "sub8_state_space.h"
#include <cmath>

using sub8::trajectory_generator::Sub8StateSpace;

ompl::base::State* Sub8StateSpace::allocState() const {
  StateType* state = new StateType();
  allocStateComponents(state);
  return state;
}

void Sub8StateSpace::freeState(State* state) const {
  CompoundStateSpace::freeState(state);
}

double Sub8StateSpace::distance(const State* state1,
                                const State* state2) const {
  // TODO
  // return LQR cost to go to state 2 from state 1? 

  // naive implementation is euclidean distance in position
  Vector3d pos1; 
  Vector3d pos2; 
  state1->as<Sub8StateSpace::StateType>()->getPosition(pos1);
  state2->as<Sub8StateSpace::StateType>()->getPosition(pos2);

  return sqrt(pow(pos2(0) - pos1(0), 2) +
              pow(pos2(1) - pos1(1), 2) + pow(pos2(2) - pos1(2), 2));
}

double Sub8StateSpace::getMaximumExtent() const {
  // TODO
  // PLACEHOLDER
  return 100;
}

bool Sub8StateSpace::equalStates(const State* state1,
                                 const State* state2) const {
  // TODO
  // Do we want to define some error-vector
  // s.t. State1 == State2 if State1 is within += error-vector of State2?
  return true;
}