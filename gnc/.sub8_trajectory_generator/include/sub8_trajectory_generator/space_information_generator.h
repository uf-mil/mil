/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#ifndef SPACE_INFORMATION_GENERATOR_H_
#define SPACE_INFORMATION_GENERATOR_H_

#include <ros/ros.h>
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"
#include "sub8_state_validity_checker.h"
#include "tgen_common.h"

using ompl::base::SpaceInformationPtr;
using ompl::base::StateSpacePtr;

namespace sub8
{
namespace trajectory_generator
{
// forward declarations for typedefs
class SpaceInformationGenerator;

// Typedefs for shared_ptr wrappers
typedef boost::shared_ptr<SpaceInformationGenerator> SpaceInformationGeneratorPtr;

class SpaceInformationGenerator
{
public:
  // method that handles instantiation and customization of the
  // SpaceInformation state and control spaces
  SpaceInformationPtr generate();

private:
  // Boundary definitions for the subspaces that
  // together make up the Sub8 state space
  void setStateSpaceBounds(const StateSpacePtr& space);
};
}
}
#endif /* SPACE_INFORMATION_GENERATOR */