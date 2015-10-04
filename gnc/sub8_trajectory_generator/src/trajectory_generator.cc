/**
* Author: Patrick Emami
* Date: 9/21/15
*/
#include <ros/ros.h>
#include "sub8_state_space.h"
#include "sub8_tgen_manager.h"
#include "sub8_msgs/MotionPlan.h"

namespace sub8 {
namespace trajectory_generator {

// Forward declaration for typedef
class TGenNode;

typedef boost::shared_ptr<TGenNode> TGenNodePtr;

// Maintains a TGenManager object and defines callbacks for service
// requests
class TGenNode {
 public:
  TGenNode(int& planner_id) : _tgen(new Sub8TGenManager(planner_id)) {}
  bool findTrajectory(sub8_msgs::MotionPlan::Request& req,
                      sub8_msgs::MotionPlan::Response& resp) {
    boost::shared_ptr<sub8_msgs::Waypoint> start_state_wpoint =
        boost::make_shared<sub8_msgs::Waypoint>(req.start_state);
    boost::shared_ptr<sub8_msgs::Waypoint> goal_state_wpoint =
        boost::make_shared<sub8_msgs::Waypoint>(req.goal_state);

    _tgen->setProblemDefinition(_tgen->waypointToState(start_state_wpoint),
                                _tgen->waypointToState(goal_state_wpoint));

    // What is the desired behavior here? If planning fails and no
    // solution path is found, should we return false from the callback? 
    // Or just set "resp.success" to false? If this happens, is it a 
    // system-shutdown-worthy event? 

    resp.success = _tgen->solve();
    if (resp.success) {
       resp.trajectory = _tgen->getTrajectory();   
    } else {
      // ALARM?
    }
    return true;
  }

 private:
  Sub8TGenManagerPtr _tgen;
};
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;

  // Grabs the planner id from the param server
  int planner_id = 0;
  nh.getParam("planner", planner_id);
  sub8::trajectory_generator::TGenNodePtr tgen(
      new sub8::trajectory_generator::TGenNode(planner_id));

  ros::ServiceServer motion_planning_srv = nh.advertiseService(
      "motion_plan", &sub8::trajectory_generator::TGenNode::findTrajectory,
      tgen);

  // Spin while listening for srv requests from the mission planner
  while (ros::ok()) {
    ros::spinOnce();
  }
}