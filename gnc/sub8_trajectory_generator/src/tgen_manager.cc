/**
 * Author: Patrick Emami
 * Date: 9/29/15
 *
 */

#include "tgen_manager.h"
#include "tgen_common.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/PathControl.h"
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <ros/package.h>
#include <Eigen/Dense>

using sub8::trajectory_generator::TGenManager;
using sub8::trajectory_generator::TGenMsgs;
using sub8::trajectory_generator::SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using sub8::trajectory_generator::SubDynamicsPtr;
using sub8::trajectory_generator::SubDynamics;
using ompl::base::Planner;
using ompl::base::PlannerStatus;
using ompl::base::GoalState;
using ompl::base::ProblemDefinitionPtr;
using ompl::control::RRT;
using ompl::control::PDST;
using ompl::control::PathControl;

namespace fs = ::boost::filesystem;

TGenManager::TGenManager(int planner_id, const Matrix2_8d& cspace_bounds,
                         TGenThrusterInfoPtr thruster_info,
                         AlarmBroadcasterPtr& alarm_broadcaster) {
  SubDynamicsPtr sub_dynamics(new SubDynamics(thruster_info));
  SpaceInformationGeneratorPtr ss_gen(new SpaceInformationGenerator());

  _sub8_si = ss_gen->generate(sub_dynamics, cspace_bounds);

  switch (planner_id) {
    case PlannerType::PDST:
      _sub8_planner =
          boost::shared_ptr<Planner>(new ompl::control::PDST(_sub8_si));
      _planner_type = PlannerType::PDST;
      break;
    default:
      _sub8_planner =
          boost::shared_ptr<Planner>(new ompl::control::RRT(_sub8_si));
      _planner_type = PlannerType::RRT;
      break;
  }

  // initialize alarms
  std::string alarms_dir;
  ros::param::get("alarms_dir", alarms_dir);
  std::string pkg_path = ros::package::getPath("sub8_trajectory_generator");
  char file_sep = '/';
#ifdef _WIN32
  file_sep = "\\";
#endif
  fs::path dirname(pkg_path + file_sep + alarms_dir);
  alarm_broadcaster->addAlarms(dirname, alarms);
}

void TGenManager::setProblemDefinition(const State* start_state,
                                       const State* goal_state) {
  if (_sub8_planner->isSetup()) {
    _sub8_planner->clear();
    ProblemDefinitionPtr pdef(new ProblemDefinition(_sub8_si));

    // set the start state for the new ProblemDefinition
    pdef->addStartState(start_state);
    // set the goal state for the new ProblemDefinition
    pdef->setGoalState(goal_state);

    _sub8_planner->setProblemDefinition(pdef);

    // Verifies that the problem definition was correctly set
    _sub8_planner->checkValidity();
  }
}

bool TGenManager::solve() {
  bool on_success = false;

  // Need to supply a real terminating condition
  PlannerStatus pstatus = _sub8_planner->solve(1.0);

  // Switch on the value of the PlannerStatus enum "StateType"
  // Only raise alarms on issues that require a system response.
  // Most errors here are due to programming errors, so just
  // display an error message to shame the programmer.
  switch (pstatus.operator StatusType()) {
    case PlannerStatus::INVALID_START:
      ROS_ERROR("%s", TGenMsgs::INVALID_START);
      break;
    case PlannerStatus::INVALID_GOAL:
      ROS_ERROR("%s", TGenMsgs::INVALID_GOAL);
      break;
    case PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
      ROS_ERROR("%s", TGenMsgs::UNRECOGNIZED_GOAL_TYPE);
      break;
    case PlannerStatus::TIMEOUT:
      ROS_ERROR("%s", TGenMsgs::TIMEOUT);
      // The logical flow here should be:
      //
      // 1. Check any safety paths?
      // 2. If no, set off ALARM
      // 3. If yes, send off safety path to controller, and then attempt to
      // replan
      //
      // Q: How many times do I retry replanning if failure? once?
      break;
    case PlannerStatus::APPROXIMATE_SOLUTION:
      ROS_DEBUG("%s", TGenMsgs::APPROXIMATE_SOLUTION);
      // TODO: What is our tolerance for this?
      on_success = true;
      break;
    case PlannerStatus::EXACT_SOLUTION:
      ROS_DEBUG("%s", TGenMsgs::EXACT_SOLUTION);
      on_success = true;
      break;
    case PlannerStatus::CRASH:
      ROS_ERROR("%s", TGenMsgs::CRASH);
      break;
    default:
      break;
  }

  return on_success;
}

void TGenManager::validateCurrentTrajectory() {
  // Planner get start state, get goal state
  unsigned int first_invalid_state = 0;

  ProblemDefinitionPtr pdef = _sub8_planner->getProblemDefinition();
  PathControl* spath = static_cast<PathControl*>(pdef->getSolutionPath().get());

  // Do trajectory validation; first_invalid_state is populated by checkMotion()
  if (!_sub8_si->checkMotion(spath->getStates(), spath->getStateCount(),
                             first_invalid_state)) {
    // Log that the current trajectory needs to be re-planned
    ROS_WARN("%s", TGenMsgs::REPLAN_NEEDED);

    // Go ahead and supply the controller with a safety path
    // at this point. For now, raise alarm to tell system to
    // stop sub motion (but not abort mission)

    // naive replanning; will implement smarter re-planning later
    State* new_start_state = spath->getState(first_invalid_state);

    // Replan from the first invalid state to the goal state
    setProblemDefinition(
        new_start_state,
        static_cast<GoalState*>(pdef->getGoal().get())->getState());
    solve();

  } else {
    ROS_INFO("%s", TGenMsgs::TRAJECTORY_VALIDATED);
  }
}

State* TGenManager::waypointToState(
    const boost::shared_ptr<sub8_msgs::Waypoint>& wpoint) {
  State* state = _sub8_si->getStateSpace()->allocState();

  state->as<Sub8StateSpace::StateType>()->setPosition(wpoint->pose.position.x,
                                                      wpoint->pose.position.y,
                                                      wpoint->pose.position.z);
  state->as<Sub8StateSpace::StateType>()->setLinearVelocity(
      wpoint->twist.linear.x, wpoint->twist.linear.y, wpoint->twist.linear.z);
  state->as<Sub8StateSpace::StateType>()->setAngularVelocity(
      wpoint->twist.angular.x, wpoint->twist.angular.y,
      wpoint->twist.angular.z);
  state->as<Sub8StateSpace::StateType>()->setOrientation(
      wpoint->pose.orientation.x, wpoint->pose.orientation.y,
      wpoint->pose.orientation.z, wpoint->pose.orientation.w);
  return state;
}

sub8_msgs::Waypoint TGenManager::stateToWaypoint(const State* state) {
  sub8_msgs::Waypoint wpoint;
  Vector13d state_vector;

  state->as<Sub8StateSpace::StateType>()->getState(state_vector);

  wpoint.pose.position.x = state_vector(0);
  wpoint.pose.position.y = state_vector(1);
  wpoint.pose.position.z = state_vector(2);
  wpoint.twist.linear.x = state_vector(3);
  wpoint.twist.linear.y = state_vector(4);
  wpoint.twist.linear.z = state_vector(5);
  wpoint.twist.angular.x = state_vector(6);
  wpoint.twist.angular.y = state_vector(7);
  wpoint.twist.angular.z = state_vector(8);
  wpoint.pose.orientation.x = state_vector(9);
  wpoint.pose.orientation.y = state_vector(10);
  wpoint.pose.orientation.z = state_vector(11);
  wpoint.pose.orientation.w = state_vector(12);

  return wpoint;
}

sub8_msgs::Trajectory TGenManager::getTrajectory() {
  std::vector<State*> states =
      (static_cast<PathControl*>(
           _sub8_planner->getProblemDefinition()->getSolutionPath().get()))
          ->getStates();

  sub8_msgs::Trajectory t_msg;

  for (State* s : states) {
    t_msg.trajectory.push_back(stateToWaypoint(s));
  }

  return t_msg;
}

void TGenManager::updatePlanningRegion(double xmin, double xmax, double ymin,
                                       double ymax, double zmin, double zmax) {
  // Bounds on position (x, y, z)
  RealVectorBounds pos_bounds(3);

  pos_bounds.setLow(0, xmin);   // x low
  pos_bounds.setHigh(0, xmax);  // x high
  pos_bounds.setLow(1, ymin);   // y low
  pos_bounds.setHigh(1, ymax);  // y high
  pos_bounds.setLow(2, zmin);   // z low
  pos_bounds.setHigh(2, zmax);  // z high

  _sub8_si->getStateSpace()
      ->as<sub8::trajectory_generator::Sub8StateSpace>()
      ->set_volume_bounds(pos_bounds);
}