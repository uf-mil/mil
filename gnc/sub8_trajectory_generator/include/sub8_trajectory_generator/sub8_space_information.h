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
using ompl::base::State;
using ompl::control::ControlSpacePtr;
using ompl::control::Control;
using ompl::control::ODESolverPtr;
using ompl::control::SpaceInformation; 

namespace sub8 {

namespace trajectory_generator {

// forward declarations for typedefs
class Sub8SpaceInformation;
class Sub8SpaceInformationGenerator;

// Typedefs for shared_ptr wrappers
typedef boost::shared_ptr<Sub8SpaceInformationGenerator>
    Sub8SpaceInformationGeneratorPtr;
typedef boost::shared_ptr<Sub8SpaceInformation> Sub8SpaceInformationPtr;

class Sub8SpaceInformationGenerator {
 public:
  // method that handles instantiation and customization of the
  // Sub8SpaceInformation state and control spaces
  Sub8SpaceInformationPtr generate();

 private:
  // Boundary definitions for the subspaces that
  // together make up the Sub8 state space
  void setStateSpaceBounds(const StateSpacePtr& space);

  // Boundary definitions for the control space-
  // e.g. thruster limitations
  void setControlSpaceBounds(const ControlSpacePtr& space);

  // Number of dimensions in the control space.
  // Currently set to 8, which is num thrusters
  const static int _CSPACE_DIMS = 8;
};

// SpaceInformation encapsulates the state and control spaces for the sub
// and provides functionality commonly used by motion planners
//
// This is really just a wrapper class for ompl::control::SpaceInformation,
// and is normally constructed by Sub8SpaceInformationGenerator, which
// handles StateSpace and ControlSpace customization
class Sub8SpaceInformation : public SpaceInformation {
 public:
  Sub8SpaceInformation(const StateSpacePtr& space,
                       const ControlSpacePtr& cspace)
      : SpaceInformation(space, cspace) {}
  // Sets the ODE solver used by the propagate method
  void setODESolver(const ODESolverPtr& ode_solver);

  // Propagate from a state, given a control, for some specified amount of time
  void propagate(const State* state, const Control* control,
                 const double duration, State* result) const;

 private:
  // Shared ptr to an ODEBasicSolver used by the propagate method
  ODESolverPtr _sub8_ode_solver;
};
}
}
#endif /* SUB8_SPACE_INFORMATION */