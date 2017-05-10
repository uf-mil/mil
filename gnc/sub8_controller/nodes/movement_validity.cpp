//Figure out what I need to include for this oh jeez.

//Gonna be using a lot of namespaces probably. 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <mil_msgs/PoseTwistStamped.h>
#include <mil_tools/msg_helpers.hpp>
#include <mil_tools/param_helpers.hpp>


#include <mil_msgs/MoveToAction.h>
#include "C3Trajectory.h"
#include "sub8_msgs/Waypoint.h"

/*
* Goal: This is going to need to check whether the current waypoint is valid. 
* Check Z coordinate of the waypoint against the current depth of the sub to determine-
* -whether this is attempting to move outside the water or not. 
* We are also going to eventually want to check whether this is going to be moving- 
* -into a wall or not.  
*/

using namespace std;
using namespace geometry_msgs;

bool isWaypointValid(const subjugator::C3Trajectory::Waypoint &wp)
{
  double wpDepth = wp.pose.point.z;
  bool resp = true;
  //stay below the surface
  ROS_INFO("request: check waypoint depth=%1d", wpDepth);
  if(wpDepth > 0)
    {
	resp = false;
    }
  ROS_INFO("response: %1d", resp);
  return resp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "isWaypointValid");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("isWaypointValid", add);
  ROS_INFO("Ready to evaluate waypoint.");
  ros::spin();

  return 0; 
}
