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

double Sub8StateSpace::distance(const State* state1_ptr,
                                const State* state2_ptr) const {
  Vector13d state1;
  Vector13d state2;
  state1_ptr->as<Sub8StateSpace::StateType>()->getState(state1);
  state2_ptr->as<Sub8StateSpace::StateType>()->getState(state2);
  
  // https://www-preview.ri.cmu.edu/pub_files/pub4/kuffner_james_2004_1/kuffner_james_2004_1.pdf
  double lambda = state1.segment(9,4).dot(state2.segment(9,4));

  // Weighted sum of distances between position, velocity, angular velocity
  // Approximate "distance" between orientations (see linked paper)
  return (distance_gain_position *
   (state2.segment(0, 3) - state1.segment(0, 3)).norm()) +
      (distance_gain_velocity * (state2.segment(3,3) - state1.segment(3,3).norm())) +
      (distance_gain_angular_velocity * (state2.segment(6,3) - state1.segment(6,3).norm())) +
      (distance_gain_orientation * (1 - abs(lambda)));
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