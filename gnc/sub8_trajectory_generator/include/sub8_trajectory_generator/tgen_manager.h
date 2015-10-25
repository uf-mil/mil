/**
 * Author: Patrick Emami
 * Date: 9/28/15
 *
 */
#ifndef _TGEN_MANAGER
#define _TGEN_MANAGER

#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/Planner.h"
#include "ompl/base/State.h"
#include "space_information_generator.h"
#include "sub8_msgs/Waypoint.h"
#include "sub8_msgs/Trajectory.h"
#include "tgen_thruster_info.h"

using ompl::base::PlannerPtr;
using ompl::base::State;
using ompl::base::ProblemDefinition;
using ompl::control::SpaceInformationPtr;
using sub8::trajectory_generator::TGenThrusterInfoPtr;

namespace sub8 {

namespace trajectory_generator {

class TGenManager;
// typedefs
typedef boost::shared_ptr<TGenManager> TGenManagerPtr;
typedef boost::shared_ptr<ProblemDefinition> ProblemDefinitionPtr;

// Mediates communication between ROS and the TGEN
class TGenManager {
 public:
  // The planner param will be passed in from the
  // param server. Instantiates a SpaceInformation
  // obj and the Planner
  TGenManager(int planner, TGenThrusterInfoPtr& thruster_info);

  // Create an ompl::base::ProblemDefinition object for planning a trajectory
  // from start_state to goal_state
  void setProblemDefinition(const State* start_state, const State* goal_state);

  // Call the Planner's solve function, returning a flag marking success
  // or failure to the caller
  bool solve();

  // The traversability map has been updated, so check whether the current
  // trajectory
  // is still valid by calling SpaceInformation's StateValidityChecker
  //
  // After issuing a "stop" trajectory, begin re-planning
  void validateCurrentTrajectory();

  /////////////////////////////////////////
  // ROS->OMPL and OMPL->ROS conversions
  /////////////////////////////////////////

  // Converts a sub8_msgs::Waypoint msg into an OMPL state object
  State* waypointToState(const boost::shared_ptr<sub8_msgs::Waypoint>& wpoint);

  // Converts an OMPL State obj to a sub8_msgs::Waypoint
  sub8_msgs::Waypoint stateToWaypoint(const State* state);

  // If a solution was found by the planner, then that
  // path is converted to a sub8_msgs::Trajectory message type
  // and returned, to be sent back in a MotionPlan service response
  //
  // To use the trajectory in Python, the caller will need to
  // convert from std::vector to list
  sub8_msgs::Trajectory getTrajectory();

 private:
  // If the current trajectory is determined to be invalid,
  // generate a new ProblemDefinition and solve again.
  //
  // On failure, send out system alarm and default to a safety-path,
  // if available. System shutdown?
  bool replan();

  PlannerPtr _sub8_planner;

  PlannerType _planner_type; 

  SpaceInformationPtr _sub8_si;

};
}
}
#endif /* _TGEN_MANAGER */