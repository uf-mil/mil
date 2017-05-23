/**
* Author: Patrick Emami
* Date: 9/21/15
*/
#include <ros/ros.h>
#include <sub8_alarm/alarm_helpers.h>
#include "sub8_msgs/Path.h"
#include "sub8_msgs/PathPlan.h"
#include "sub8_msgs/PathPoint.h"
#include "tgen_common.h"
#include "tgen_manager.h"

using sub8::AlarmBroadcasterPtr;
using sub8::AlarmBroadcaster;

namespace sub8
{
namespace trajectory_generator
{
// Forward declaration for typedef
class TGenNode;
typedef boost::shared_ptr<TGenNode> TGenNodePtr;

// Maintains a TGenManager object and defines callbacks for service
// requests
class TGenNode
{
public:
  TGenNode(AlarmBroadcasterPtr&& ab) : _tgen(new TGenManager(std::move(ab)))
  {
  }

  bool findPath(sub8_msgs::PathPlan::Request& req, sub8_msgs::PathPlan::Response& resp)
  {
    geometry_msgs::Pose start_state = req.start_state;
    geometry_msgs::Pose goal_state = req.goal_state;

    if (!(_tgen->setProblemDefinition(_tgen->poseToState(start_state), _tgen->poseToState(goal_state))))
    {
      // throw an error here?
      ROS_ERROR("Failed to set problem definition");
      return false;
    }
    // add header
    resp.success = _tgen->solve();
    if (resp.success)
    {
      ROS_INFO("Successfully found a trajectory");
      resp.path = _tgen->generatePathMessage();
      resp.path.header.stamp = ros::Time::now();
    }
    return resp.success;
  }

private:
  TGenManagerPtr _tgen;
};
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub8_trajectory_generator");
  ros::NodeHandle nh;  // start the node

  ROS_INFO_STREAM("Initializing sub8_trajectory_generator");

  ////////////////////////////////////////////
  // Initialize the TGEN node
  ////////////////////////////////////////////

  // Create the TGEN node
  sub8::trajectory_generator::TGenNodePtr tgen(new sub8::trajectory_generator::TGenNode(
      std::move(AlarmBroadcasterPtr(new AlarmBroadcaster(boost::make_shared<ros::NodeHandle>(nh))))));

  // Register the motion_plan service
  ros::ServiceServer motion_planning_srv =
      nh.advertiseService("sub8_trajectory_generator/path_plan", &sub8::trajectory_generator::TGenNode::findPath, tgen);

  // Spin while listening for srv requests from the mission planner
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}