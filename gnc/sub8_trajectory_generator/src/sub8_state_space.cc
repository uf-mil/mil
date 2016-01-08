/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */

#include "sub8_state_space.h"
#include <iostream>
#include <cmath>

static const double MAX_QUATERNION_ERROR = 1e-9;

using sub8::trajectory_generator::Sub8StateSpace;

ompl::base::State* Sub8StateSpace::allocState() const {
  StateType* state = new StateType();
  allocStateComponents(state);
  return state;
}

void Sub8StateSpace::freeState(State* state) const {
  CompoundStateSpace::freeState(state);
}

double Sub8StateSpace::distance(const State* a, const State* b) const {
  Vector13d state_a = a->as<Sub8StateSpace::StateType>()->getState();
  Vector13d state_b = b->as<Sub8StateSpace::StateType>()->getState();

  // https://www-preview.ri.cmu.edu/pub_files/pub4/kuffner_james_2004_1/kuffner_james_2004_1.pdf
  double lambda = state_a.segment(9, 4).dot(state_b.segment(9, 4));

  // Weighted sum of distances between position, velocity, angular velocity &
  // Approximate "distance" metric between orientations; a.k.a angle between
  // orientations
  return (weights_[0] *
          (state_b.segment(0, 3) - state_a.segment(0, 3)).norm()) +
         (weights_[1] *
          (state_b.segment(3, 3) - state_a.segment(3, 3)).norm()) +
         (weights_[2] *
          (state_b.segment(6, 3) - state_a.segment(6, 3)).norm()) +
         (weights_[3] * acos(fabs(lambda)));
}

//double Sub8StateSpace::getMaximumExtent() const { return INFINITY; }

bool Sub8StateSpace::equalStates(const State* a, const State* b) const {
  double error = 1e-4;

  Vector13d state_a = a->as<Sub8StateSpace::StateType>()->getState();
  Vector13d state_b = b->as<Sub8StateSpace::StateType>()->getState();

  // compare position, velocity, and angular velocity
  for (unsigned int i = 0; i < 9; ++i) {
    if (fabs(state_a(i) - state_b(i)) > error) {
      return false;
    }
  }

  // compare orientation
  // dot of two similar orientations should be 1
  if (fabs(state_a.segment(9, 4).dot(state_b.segment(9, 4))) <
      (1.0 - MAX_QUATERNION_ERROR)) {
    return false;
  }

  return true;
}

void Sub8StateSpace::enforceBounds(State* state) const {
  SO3StateSpace so3;
  double qn = so3.norm(
      state->as<Sub8StateSpace::StateType>()->as<SO3StateSpace::StateType>(3));

  if ((qn < 1 - MAX_QUATERNION_ERROR) || (qn > 1 + MAX_QUATERNION_ERROR)) {
    Vector4d q = state->as<Sub8StateSpace::StateType>()->getOrientation() / qn;
    state->as<Sub8StateSpace::StateType>()->setOrientation(q(0), q(1), q(2),
                                                           q(3));
  }
}