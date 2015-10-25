/**
* Author: Patrick Emami
* Date: 9/22/15
*
*/
#ifndef SUB8_STATE_VALIDITY_CHECKER_H_
#define SUB8_STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "tgen_common.h"

using ompl::base::StateValidityChecker;
using ompl::base::Path;
using ompl::base::State;
using ompl::control::SpaceInformationPtr;

namespace sub8 {

namespace trajectory_generator {

// forward declaration for the typedef
class Sub8StateValidityChecker;

// Typedef for shared_ptr wrapper
typedef boost::shared_ptr<Sub8StateValidityChecker> Sub8StateValidityCheckerPtr;

// Encapsulates collision checking and other functionality
// necessary for determining whether a trajectory is safe
class Sub8StateValidityChecker : public StateValidityChecker {
 public:
  Sub8StateValidityChecker(const SpaceInformationPtr& si)
      : StateValidityChecker(si) {}

  /////////////////////////////////////////////////////
  // Inherited methods
  ////////////////////////////////////////////////////

  // Return true if the state is valid. Usually,
  // this means at least collision checking. If it is
  // possible that ompl::base::StateSpace::interpolate()
  // or ompl::control::ControlSpace::propagate() return states
  // that are outside of bounds, this function should also make
  // a call to ompl::control::SpaceInformation::satisfiesBounds().
  virtual bool isValid(const State* state) const;

  // Return true if the state state is valid. In addition,
  // set dist to the distance to the nearest invalid state.
  virtual bool isValid(const State* state, double& dist) const;

  // Report the distance to the nearest invalid state
  // when starting from state. If the distance is negative,
  // the value of clearance is the penetration depth.
  virtual double clearance(const State* state) const;
};
}
}
#endif /* SUB8_STATE_VALIDITY_CHECKER */