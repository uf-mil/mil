/**
* Author: Patrick Emami
* Date: 9/21/15
*/
#include <ros/ros.h>
#include "sub8_state_space.h"
#include "tgen_thruster_info.h"
#include "tgen_manager.h"
#include "tgen_common.h"
#include <sub8_alarm/alarm_helpers.h>
#include "sub8_msgs/MotionPlan.h"
#include "sub8_msgs/ThrusterInfo.h"
#include "sub8_msgs/BMatrix.h"

using sub8::trajectory_generator::Matrix2_8d;
using sub8::trajectory_generator::_THRUSTERS_ID_BEGIN;
using sub8::trajectory_generator::_THRUSTERS_ID_END;
using sub8::trajectory_generator::TGenThrusterInfoPtr;
using sub8::trajectory_generator::TGenThrusterInfo;
using sub8::AlarmBroadcasterPtr;
using sub8::AlarmBroadcaster;

namespace sub8 {
namespace trajectory_generator {

// Forward declaration for typedef
class TGenNode;

typedef boost::shared_ptr<TGenNode> TGenNodePtr;

// Maintains a TGenManager object and defines callbacks for service
// requests
class TGenNode {
 public:
  TGenNode(int& planner_id, const Matrix2_8d& cspace_bounds,
           TGenThrusterInfoPtr& thruster_info, AlarmBroadcasterPtr& ab)
      : _tgen(new TGenManager(planner_id, cspace_bounds, thruster_info, ab)) {}

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
      // Robot must make some decisions:
      // 1. Try finding another trajectory
      // 2. Try to navigate with dead-reckoning
      //    e.g. (Do a simplistic calculation to do
      //     navigation without 6DOF
      //     motion-planning/Traversability map)
      // 3. Abort mission
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

  // Local vars
  int planner_id = 0;
  int dim_idx = 0;
  Matrix2_8d cspace_bounds;  // 2 x 8 Matrix <doubles> - thruster force ranges
  sub8_msgs::BMatrix B;      // B matrix for sub dynamics
  TGenThrusterInfoPtr thruster_info(
      new TGenThrusterInfo());  // stores info on the sub's thrusters

  // Services needed to configure the TGEN
  std::string thruster_range_service = "thrusters/thruster_range";
  std::string b_matrix_service = "b_matrix";

  // Grabs the planner id from the param server
  ros::param::get("/planner", planner_id);

  // NodeHandle
  ros::NodeHandle nh;

  ////////////////////////////////////////////
  // ServiceClient: thrusters/thruster_range
  ////////////////////////////////////////////
  ros::ServiceClient thrusters_srv_client =
      nh.serviceClient<sub8_msgs::ThrusterInfo>(thruster_range_service);
  // Alert that this node will block until this service is available
  ROS_WARN("Waiting for service %s", thruster_range_service.c_str());
  thrusters_srv_client.waitForExistence();

  for (int i = _THRUSTERS_ID_BEGIN; i <= _THRUSTERS_ID_END; ++i) {
    sub8_msgs::ThrusterInfo ti;
    ti.request.thruster_id = i;
    ROS_WARN("Retrieving thruster ranges for thruster %d", i);
    if (thrusters_srv_client.call(ti)) {
      cspace_bounds(0, dim_idx) = ti.response.min_force;
      cspace_bounds(1, dim_idx) = ti.response.max_force;
    } else {
      ROS_ERROR("Could not get thruster ranges for thruster %d", i);
      return 1;
    }
    ++dim_idx;
  }

  ////////////////////////////////////////////
  // ServiceClient: B matrix
  ////////////////////////////////////////////
  ros::ServiceClient b_matrix_service_client =
      nh.serviceClient<sub8_msgs::BMatrix>(b_matrix_service);

  // Alert that this node will block until this service is available
  ROS_WARN("Waiting for service %s", b_matrix_service.c_str());
  b_matrix_service_client.waitForExistence();
  ROS_WARN("Retrieving %s", b_matrix_service.c_str());

  if (b_matrix_service_client.call(B)) {
    // unpack B matrix
    thruster_info->init(B.response.B);
  } else {
    ROS_ERROR("Could not get %s", b_matrix_service.c_str());
    // raise alarm!
  }

  ////////////////////////////////////////////
  // Initialize the TGEN node
  ////////////////////////////////////////////

  AlarmBroadcasterPtr alarm_b(
      new AlarmBroadcaster(boost::make_shared<ros::NodeHandle>(nh)));

  // Create the TGEN node
  sub8::trajectory_generator::TGenNodePtr tgen(
      new sub8::trajectory_generator::TGenNode(planner_id, cspace_bounds,
                                               thruster_info, alarm_b));

  // Register the motion_plan service
  ros::ServiceServer motion_planning_srv = nh.advertiseService(
      "motion_plan", &sub8::trajectory_generator::TGenNode::findTrajectory,
      tgen);

  // Spin while listening for srv requests from the mission planner
  while (ros::ok()) {
    ros::spinOnce();
  }
}