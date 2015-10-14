/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#ifndef SUB8_STATE_SPACE_INFORMATION_H_
#define SUB8_STATE_SPACE_INFORMATION_H_

#include "sub8_state_validity_checker.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/ODESolver.h"
#include "ompl/control/SpaceInformation.h"
#include "sub8_state_space.h"

using ompl::base::StateSpacePtr;
using ompl::control::ControlSpacePtr;
using ompl::control::ODESolverPtr;
using ompl::control::SpaceInformationPtr; 

namespace sub8 {

namespace trajectory_generator {

// forward declarations for typedefs
class Sub8SpaceInformationGenerator;

// Typedefs for shared_ptr wrappers
typedef boost::shared_ptr<Sub8SpaceInformationGenerator>
    Sub8SpaceInformationGeneratorPtr;

class Sub8SpaceInformationGenerator {
 public:
  // method that handles instantiation and customization of the
  // Sub8SpaceInformation state and control spaces
  SpaceInformationPtr generate();

 private:
  // Boundary definitions for the subspaces that
  // together make up the Sub8 state space
  void setStateSpaceBounds(const StateSpacePtr& space);

  // Boundary definitions for the control space-
  // e.g. thruster limitations
  void setControlSpaceBounds(const ControlSpacePtr& space);
};
}
}
#endif /* SUB8_SPACE_INFORMATION */