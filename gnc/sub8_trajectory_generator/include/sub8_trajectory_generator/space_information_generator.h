/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#ifndef SPACE_INFORMATION_GENERATOR_H_
#define SPACE_INFORMATION_GENERATOR_H_

#include <ros/ros.h>
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/ODESolver.h"
#include "ompl/control/SpaceInformation.h"
#include "sub8_state_validity_checker.h"
#include "sub8_state_space.h"
#include "sub_dynamics.h"

using ompl::base::StateSpacePtr;
using ompl::control::ControlSpacePtr;
using ompl::control::ODESolverPtr;
using ompl::control::SpaceInformationPtr; 
using sub8::trajectory_generator::SubDynamicsPtr; 

namespace sub8 {

namespace trajectory_generator {

// forward declarations for typedefs
class SpaceInformationGenerator;

// Typedefs for shared_ptr wrappers
typedef boost::shared_ptr<SpaceInformationGenerator>
    SpaceInformationGeneratorPtr;

class SpaceInformationGenerator {
 public:
  // method that handles instantiation and customization of the
  // SpaceInformation state and control spaces
  SpaceInformationPtr generate(SubDynamicsPtr& sub_dynamics);

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
#endif /* SPACE_INFORMATION_GENERATOR */