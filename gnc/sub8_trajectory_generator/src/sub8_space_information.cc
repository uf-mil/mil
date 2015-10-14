/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "sub8_space_information.h"
#include "sub8_state_validity_checker.h"
#include "sub8_ode_solver.h"
#include "sub8_tgen_common.h"
#include "ompl/control/ODESolver.h"

using sub8::trajectory_generator::Sub8SpaceInformationGenerator;
using sub8::trajectory_generator::Sub8StateValidityChecker;
using sub8::trajectory_generator::Sub8StateValidityCheckerPtr;
using ompl::control::SpaceInformationPtr;

SpaceInformationPtr Sub8SpaceInformationGenerator::generate() {
  StateSpacePtr space(new Sub8StateSpace());
  // Set bounds for Sub8StateSpace
  setStateSpaceBounds(space);

  ControlSpacePtr cspace(
      new ompl::control::RealVectorControlSpace(space, _CSPACE_DIMS));
  // Set bounds for cspace
  setControlSpaceBounds(cspace);

  SpaceInformationPtr si_ptr(
      new ompl::control::SpaceInformation(space, cspace));

  // Create and set the state validity checker
  Sub8StateValidityCheckerPtr vc_ptr(new Sub8StateValidityChecker(si_ptr));
  si_ptr->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(vc_ptr));
  si_ptr->setStateValidityCheckingResolution(0.03);  // 3 % -- TODO

  // Create and set the StatePropagator with our ODESolver
  ODESolverPtr ode_solver(
      new ompl::control::ODEBasicSolver<>(si_ptr, &sub8ODE));
  si_ptr->setStatePropagator(
      ompl::control::ODESolver::getStatePropagator(ode_solver));

  // TODO -- placeholder values
  //
  // When controls are applied to states, they are applied for a time duration
  // that is an integer multiple of the stepSize, within the bounds specified by
  // setMinMaxControlDuration()
  si_ptr->setPropagationStepSize(2.0);
  si_ptr->setMinMaxControlDuration(1.0, 100.0);

  // Must be run once before use
  si_ptr->setup();

  return si_ptr;
}

void Sub8SpaceInformationGenerator::setStateSpaceBounds(
    const StateSpacePtr& space) {
  // TODO
  // When setting start and goal states, when are those states validated with
  // the state space's bounds?

  // Bounds on position (x, y, z)
  RealVectorBounds pos_bounds(3);
  // Bounds on velocity (x, y, z)
  RealVectorBounds vel_bounds(3);
  // Bounds on angular velocity (wx, wy, wz)
  RealVectorBounds w_bounds(3);
  // Bounds on orientation (qx, qy, qz, qw)
  RealVectorBounds q_bounds(4);

  // THESE ARE PLACEHOLDERS -- NEED TO SET THESE TO THE SUB's ACTUAL BOUNDS
  // LATER

  pos_bounds.setLow(0, -100);  // x low
  pos_bounds.setHigh(0, 100);  // x high
  pos_bounds.setLow(1, -100);  // y low
  pos_bounds.setHigh(1, 100);  // y high
  pos_bounds.setLow(2, -100);  // z low
  pos_bounds.setHigh(2, 100);  // z high

  vel_bounds.setLow(0, -5);  // x low
  vel_bounds.setHigh(0, 5);  // x high
  vel_bounds.setLow(1, -5);  // y low
  vel_bounds.setHigh(1, 5);  // y high
  vel_bounds.setLow(2, -5);  // z low
  vel_bounds.setHigh(2, 5);  // z high

  w_bounds.setLow(0, -3);  // x low
  w_bounds.setHigh(0, 3);  // x high
  w_bounds.setLow(1, -3);  // y low
  w_bounds.setHigh(1, 3);  // y high
  w_bounds.setLow(2, -3);  // z low
  w_bounds.setHigh(2, 3);  // z high

  q_bounds.setLow(0, -1);  // x low
  q_bounds.setHigh(0, 1);  // x high
  q_bounds.setLow(1, -1);  // y low
  q_bounds.setHigh(1, 1);  // y high
  q_bounds.setLow(2, -1);  // z low
  q_bounds.setHigh(2, 1);  // z high
  q_bounds.setLow(3, -1);  // w low
  q_bounds.setHigh(3, 1);  // w high

  space->as<sub8::trajectory_generator::Sub8StateSpace>()->set_volume_bounds(
      pos_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_linear_velocity_bounds(vel_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_angular_velocity_bounds(w_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_orientation_bounds(q_bounds);
}

void Sub8SpaceInformationGenerator::setControlSpaceBounds(
    const ControlSpacePtr& space) {
  // TODO
}
