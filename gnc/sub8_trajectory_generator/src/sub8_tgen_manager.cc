/**
 * Author: Patrick Emami
 * Date: 9/29/15
 *
 */

#include "sub8_tgen_manager.h"
#include "sub8_tgen_common.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/PathControl.h"

#include <ros/console.h>

using sub8::trajectory_generator::Sub8TGenManager;
using sub8::trajectory_generator::Sub8TGenMsgs;
using sub8::trajectory_generator::Sub8SpaceInformationGeneratorPtr;
using sub8::trajectory_generator::Sub8StateSpace;
using ompl::base::Planner;
using ompl::base::PlannerStatus;
using ompl::base::GoalState;
using ompl::base::ProblemDefinitionPtr;
using ompl::control::RRT;
using ompl::control::PDST;
using ompl::control::PathControl;

Sub8TGenManager::Sub8TGenManager(int planner) {
  Sub8SpaceInformationGeneratorPtr ss_gen(new Sub8SpaceInformationGenerator());
  _sub8_si = ss_gen->generate();

  // Instantiate the planner indicated in the launch file
  // Switch on the PlannerType
  switch (planner) {
    case PlannerType::PDST:
      _sub8_planner =
          boost::shared_ptr<Planner>(new ompl::control::PDST(_sub8_si));
      break;
    default:
      _sub8_planner =
          boost::shared_ptr<Planner>(new ompl::control::RRT(_sub8_si));
      break;
  }
}

void Sub8TGenManager::setProblemDefinition(const State* start_state,
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

bool Sub8TGenManager::solve() {
  bool on_success = false;

  // Need to supply a real terminating condition
  PlannerStatus pstatus = _sub8_planner->solve(1.0);

  // Switch on the value of the PlannerStatus enum "StateType"
  switch (pstatus.operator StatusType()) {
    case PlannerStatus::INVALID_START:
      ROS_ERROR("%s", Sub8TGenMsgs::INVALID_START);
      // TODO - ALARM
      break;
    case PlannerStatus::INVALID_GOAL:
      ROS_ERROR("%s", Sub8TGenMsgs::INVALID_GOAL);
      // TODO - ALARM
      break;
    case PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
      ROS_ERROR("%s", Sub8TGenMsgs::UNRECOGNIZED_GOAL_TYPE);
      // TODO - ALARM
      break;
    case PlannerStatus::TIMEOUT:
      ROS_ERROR("%s", Sub8TGenMsgs::TIMEOUT);
      // The logical flow here should be:
      //
      // 1. Check any safety paths?
      // 2. If no, set off ALARM
      // 3. If yes, send off safety path to controller, and then attempt to
      // replan
      //
      // Q: How many times do I retry replanning if failure?
      break;
    case PlannerStatus::APPROXIMATE_SOLUTION:
      ROS_DEBUG("%s", Sub8TGenMsgs::APPROXIMATE_SOLUTION);
      on_success = true;
      break;
    case PlannerStatus::EXACT_SOLUTION:
      ROS_DEBUG("%s", Sub8TGenMsgs::EXACT_SOLUTION);
      on_success = true;
      break;
    case PlannerStatus::CRASH:
      ROS_ERROR("%s", Sub8TGenMsgs::CRASH);
      // ALARM
      break;
    default:
      break;
  }

  return on_success;
}

void Sub8TGenManager::validateCurrentTrajectory() {
  // Planner get start state, get goal state
  unsigned int first_invalid_state = 0;

  ProblemDefinitionPtr pdef = _sub8_planner->getProblemDefinition();
  PathControl* spath = static_cast<PathControl*>(pdef->getSolutionPath().get());

  if (!_sub8_si->checkMotion(spath->getStates(), spath->getStateCount(),
                             first_invalid_state)) {
    ROS_WARN("%s", Sub8TGenMsgs::REPLAN_FAILED);
    // Go ahead and supply the controller with a safety path

    // naive replanning; will implement smarter re-planning later
    State* new_start_state = spath->getState(first_invalid_state);

    // Replan from the first invalid state to the goal state
    setProblemDefinition(
        new_start_state,
        static_cast<GoalState*>(pdef->getGoal().get())->getState());
    solve();

  } else {
    ROS_INFO("%s", Sub8TGenMsgs::TRAJECTORY_VALIDATED);
  }
}

State* Sub8TGenManager::waypointToState(
    const boost::shared_ptr<sub8_msgs::Waypoint>& wpoint) {
  State* state = _sub8_si->getStateSpace()->allocState();

  state->as<Sub8StateSpace::StateType>()->setPosition(
      wpoint->pos.position.x, wpoint->pos.position.y, wpoint->pos.position.z);
  state->as<Sub8StateSpace::StateType>()->setLinearVelocity(
      wpoint->vel.linear.x, wpoint->vel.linear.y, wpoint->vel.linear.z);
  state->as<Sub8StateSpace::StateType>()->setAngularVelocity(
      wpoint->vel.angular.x, wpoint->vel.angular.y, wpoint->vel.angular.z);
  state->as<Sub8StateSpace::StateType>()->setOrientation(
      wpoint->pos.orientation.x, wpoint->pos.orientation.y,
      wpoint->pos.orientation.z, wpoint->pos.orientation.w);
  return state;
}

sub8_msgs::Waypoint Sub8TGenManager::stateToWaypoint(const State* state) {
  sub8_msgs::Waypoint wpoint;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> w;
  std::vector<double> orientation;

  // Initalize the vectors with the state data
  state->as<Sub8StateSpace::StateType>()->getPosition(pos);
  state->as<Sub8StateSpace::StateType>()->getLinearVelocity(vel);
  state->as<Sub8StateSpace::StateType>()->getAngularVelocity(w);
  state->as<Sub8StateSpace::StateType>()->getOrientation(orientation);

  wpoint.pos.position.x = pos[0];
  wpoint.pos.position.y = pos[1];
  wpoint.pos.position.z = pos[2];
  wpoint.vel.linear.x = vel[0];
  wpoint.vel.linear.y = vel[1];
  wpoint.vel.linear.z = vel[2];
  wpoint.vel.angular.x = w[0];
  wpoint.vel.angular.y = w[1];
  wpoint.vel.angular.z = w[2];
  wpoint.pos.orientation.x = orientation[0];
  wpoint.pos.orientation.y = orientation[1];
  wpoint.pos.orientation.z = orientation[2];
  wpoint.pos.orientation.w = orientation[3];
  return wpoint;
}

sub8_msgs::Trajectory Sub8TGenManager::getTrajectory() {
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