/**
* Author: Patrick Emami
* Date: 9/22/15
*/
#include "space_information_generator.h"
#include "sub8_state_validity_checker.h"
#include "ompl/control/ODESolver.h"
#include "sub8_msgs/ThrusterInfo.h"
#include <ros/console.h>

using sub8::trajectory_generator::SpaceInformationGenerator;
using sub8::trajectory_generator::Sub8StateValidityChecker;
using sub8::trajectory_generator::Sub8StateValidityCheckerPtr;
using ompl::control::SpaceInformationPtr;

SpaceInformationPtr SpaceInformationGenerator::generate(
    const SubDynamicsPtr& sub_dynamics, const Matrix2_8d& cspace_bounds) {
  // Grab config parameters
  double checking_res;
  double prop_step_size;
  double min_control_duration;
  double max_control_duration;
  ros::param::get("state_validity_checking_resolution", checking_res);
  ros::param::get("propagation_step_size", prop_step_size);
  ros::param::get("min_control_duration", min_control_duration);
  ros::param::get("max_control_duration", max_control_duration);

  StateSpacePtr space(new Sub8StateSpace());
  // Set bounds for Sub8StateSpace
  setStateSpaceBounds(space);

  ControlSpacePtr cspace(
      new ompl::control::RealVectorControlSpace(space, _CSPACE_DIMS));
  // Set bounds for cspace
  setControlSpaceBounds(cspace, cspace_bounds);

  SpaceInformationPtr si_ptr(
      new ompl::control::SpaceInformation(space, cspace));

  // Create and set the state validity checker
  Sub8StateValidityCheckerPtr vc_ptr(new Sub8StateValidityChecker(si_ptr));
  si_ptr->setStateValidityChecker(
      static_cast<ompl::base::StateValidityCheckerPtr>(vc_ptr));
  si_ptr->setStateValidityCheckingResolution(checking_res);

  // Create and set the StatePropagator with our ODESolver
  ODESolverPtr ode_solver(new ompl::control::ODEBasicSolver<>(
      si_ptr, boost::bind(&SubDynamics::ode, sub_dynamics, _1, _2, _3)));
  si_ptr->setStatePropagator(ompl::control::ODESolver::getStatePropagator(
      ode_solver,
      boost::bind(&SubDynamics::postPropagate, sub_dynamics, _1, _2, _3, _4)));

  // TODO -- placeholder values
  //
  // When controls are applied to states, they are applied for a time duration
  // that is an integer multiple of the stepSize, within the bounds specified by
  // setMinMaxControlDuration()
  si_ptr->setPropagationStepSize(prop_step_size);
  si_ptr->setMinMaxControlDuration(min_control_duration, max_control_duration);

  // Must be run once before use
  si_ptr->setup();

  return si_ptr;
}

void SpaceInformationGenerator::setStateSpaceBounds(
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

  ros::param::get("xmin", pos_bounds.low[0]);
  ros::param::get("xmax", pos_bounds.high[0]);
  ros::param::get("ymin", pos_bounds.low[1]);
  ros::param::get("ymax", pos_bounds.high[1]);
  ros::param::get("zmin", pos_bounds.low[2]);
  ros::param::get("zmax", pos_bounds.high[2]);

  ros::param::get("v_xmin", vel_bounds.low[0]);
  ros::param::get("v_xmax", vel_bounds.high[0]);
  ros::param::get("v_ymin", vel_bounds.low[1]);
  ros::param::get("v_ymax", vel_bounds.high[1]);
  ros::param::get("v_zmin", vel_bounds.low[2]);
  ros::param::get("v_zmax", vel_bounds.high[2]);

  ros::param::get("w_xmin", w_bounds.low[0]);
  ros::param::get("w_xmax", w_bounds.high[0]);
  ros::param::get("w_ymin", w_bounds.low[1]);
  ros::param::get("w_ymax", w_bounds.high[1]);
  ros::param::get("w_zmin", w_bounds.low[2]);
  ros::param::get("w_zmax", w_bounds.high[2]);

  ros::param::get("q_xmin", q_bounds.low[0]);
  ros::param::get("q_xmax", q_bounds.high[0]);
  ros::param::get("q_ymin", q_bounds.low[1]);
  ros::param::get("q_ymax", q_bounds.high[1]);
  ros::param::get("q_zmin", q_bounds.low[2]);
  ros::param::get("q_zmax", q_bounds.high[2]);
  ros::param::get("q_wmin", q_bounds.low[2]);
  ros::param::get("q_wmax", q_bounds.high[2]);

  space->as<sub8::trajectory_generator::Sub8StateSpace>()->set_volume_bounds(
      pos_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_linear_velocity_bounds(vel_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_angular_velocity_bounds(w_bounds);
  space->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_orientation_bounds(q_bounds);
}

void SpaceInformationGenerator::setControlSpaceBounds(
    const ControlSpacePtr& space, const Matrix2_8d& cspace_bounds) {
  RealVectorBounds bounds(8);

  bounds.setLow(0, cspace_bounds(0, 0));
  bounds.setHigh(0, cspace_bounds(1, 0));
  bounds.setLow(1, cspace_bounds(0, 1));
  bounds.setHigh(1, cspace_bounds(1, 1));
  bounds.setLow(2, cspace_bounds(0, 2));
  bounds.setHigh(2, cspace_bounds(1, 2));
  bounds.setLow(3, cspace_bounds(0, 3));
  bounds.setHigh(3, cspace_bounds(1, 3));
  bounds.setLow(4, cspace_bounds(0, 4));
  bounds.setHigh(4, cspace_bounds(1, 4));
  bounds.setLow(5, cspace_bounds(0, 5));
  bounds.setHigh(5, cspace_bounds(1, 5));
  bounds.setLow(6, cspace_bounds(0, 6));
  bounds.setHigh(6, cspace_bounds(1, 6));
  bounds.setLow(7, cspace_bounds(0, 7));
  bounds.setHigh(7, cspace_bounds(1, 7));

  space->as<ompl::control::RealVectorControlSpace>()->setBounds(bounds);
}
