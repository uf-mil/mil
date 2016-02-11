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
#include "sub8_msgs/Path.h"
#include "sub8_msgs/PathPoint.h"
#include "sub8_alarm/alarm_helpers.h"
#include <geometry_msgs/Pose.h>
#include <string>
#include <map>
#define eps_ 0.001

using ompl::base::PlannerPtr;
using ompl::base::State;
using ompl::base::ProblemDefinition;
using ompl::base::SpaceInformationPtr;
using sub8::AlarmBroadcasterPtr;

namespace sub8 {

namespace trajectory_generator {

class TGenManager;
// typedefs
typedef boost::shared_ptr<TGenManager> TGenManagerPtr;
typedef boost::shared_ptr<ProblemDefinition> ProblemDefinitionPtr;

// Mediates communication between ROS and the TGEN
class TGenManager {
 public:

  TGenManager(AlarmBroadcasterPtr&& ab);

  // Create an ompl::base::ProblemDefinition object for planning a path
  // from start_state to goal_state
  //
  // returns false if setting the problem definition failed due to providing
  // invalid start or goal states
  bool setProblemDefinition(const State* start_state, const State* goal_state);

  // Call the Planner's solve function, returning a flag marking success
  // or failure to the caller
  bool solve();

  // The traversability map has been updated, so check whether the current
  // path is still valid by calling SpaceInformation's StateValidityChecker
  //
  // After issuing a "stop", begin re-planning
  void validateCurrentPath();

  /////////////////////////////////////////
  // ROS->OMPL and OMPL->ROS conversions
  /////////////////////////////////////////

  // Converts a geometry_msgs::Pose into an OMPL state object
  State* poseToState(const geometry_msgs::Pose& pose);

  // Converts an OMPL State obj to a sub8_msgs::PathPoint path_point
  sub8_msgs::PathPoint stateToPathPoint(const State* state);

  // If a solution was found by the planner, then that
  // path is converted to a sub8_msgs::Path message type
  // and returned, to be sent back in a PathPlan service response
  //
  // To use the trajectory in Python, the caller will need to
  // convert from std::vector to list
  sub8_msgs::Path generatePathMessage();

  // Return the current trajectory as a vector of pointers to States
  std::vector<State*> getPath();

 private:
  bool approx(double x, double y) {
    return ((x < y + eps_) && (x > y - eps_)) ? true : false;
  }

  PlannerPtr _sub8_planner;

  ProblemDefinitionPtr _pdef;

  SpaceInformationPtr _sub8_si;

  AlarmBroadcasterPtr _alarm_broadcaster; 
  
  AlarmRaiserPtr _planning_failure_alarm;
};
}
}
#endif /* _TGEN_MANAGER */