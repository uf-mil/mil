/**
* Author: Patrick Emami
* Date: 9/21/15
*/
#include <ros/ros.h>
#include "sub8_state_space.h"
#include "tgen_manager.h"
#include "tgen_common.h"
#include "sub8_msgs/MotionPlan.h"
#include "sub8_msgs/ThrusterInfo.h"

using sub8::trajectory_generator::Matrix2_8d;
using sub8::trajectory_generator::_THRUSTERS_ID_BEGIN;
using sub8::trajectory_generator::_THRUSTERS_ID_END;

namespace sub8 {
namespace trajectory_generator {

// Forward declaration for typedef
class TGenNode;

typedef boost::shared_ptr<TGenNode> TGenNodePtr;

// Maintains a TGenManager object and defines callbacks for service
// requests
class TGenNode {
 public:
  TGenNode(int& planner_id, const Matrix2_8d& cspace_bounds)
      : _tgen(new TGenManager(planner_id, cspace_bounds)) {}
  bool findTrajectory(sub8_msgs::MotionPlan::Request& req,
                      sub8_msgs::MotionPlan::Response& resp) {
    boost::shared_ptr<sub8_msgs::Waypoint> start_state_wpoint =
        boost::make_shared<sub8_msgs::Waypoint>(req.start_state);
    boost::shared_ptr<sub8_msgs::Waypoint> goal_state_wpoint =
        boost::make_shared<sub8_msgs::Waypoint>(req.goal_state);

    _tgen->setProblemDefinition(_tgen->waypointToState(start_state_wpoint),
                                _tgen->waypointToState(goal_state_wpoint));

    resp.success = _tgen->solve();
    if (resp.success) {
      resp.trajectory = _tgen->getTrajectory();
    } else {
      // ALARM?
    }
    return true;
  }

 private:
  TGenManagerPtr _tgen;
};
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;

  // Grabs the planner id from the param server
  int planner_id = 0;
  nh.getParam("planner", planner_id);

  ros::ServiceClient sc =
      nh.serviceClient<sub8_msgs::ThrusterInfo>("thrusters/thruster_range");
  // Block until this service is available
  sc.waitForExistence(); 

  int dim_idx = 0;
  Matrix2_8d cspace_bounds;
  for (int i = _THRUSTERS_ID_BEGIN; i <= _THRUSTERS_ID_END; ++i) {
    sub8_msgs::ThrusterInfo ti;
    ti.request.thruster_id = i;
    ROS_WARN("Retrieving thruster ranges for thruster %d", i);
    if (sc.call(ti)) {
      cspace_bounds(0, dim_idx) = ti.response.min_force;
      cspace_bounds(1, dim_idx) = ti.response.max_force;
    } else {
      ROS_ERROR("Could not get thruster ranges for thruster %d", i);
      return 1;
    }
    ++dim_idx;
  }

  sub8::trajectory_generator::TGenNodePtr tgen(
      new sub8::trajectory_generator::TGenNode(planner_id, cspace_bounds));

  ros::ServiceServer motion_planning_srv = nh.advertiseService(
      "motion_plan", &sub8::trajectory_generator::TGenNode::findTrajectory,
      tgen);

  // Spin while listening for srv requests from the mission planner
  while (ros::ok()) {
    ros::spinOnce();
  }
}