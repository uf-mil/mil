#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "sub8_msgs/WaypointValidity.h"

/*
* Goal: This is going to need to check whether the current waypoint is valid.
* Check Z coordinate of the waypoint against the current depth of the sub to determine-
* -whether this is attempting to move outside the water or not.
* We are also going to eventually want to check whether this is going to be moving-
* -into a wall or not.
*/

using namespace std;
using namespace geometry_msgs;

// Todo: Implement check for ogrid
bool isWaypointValid(sub8_msgs::WaypointValidity::Request &req, sub8_msgs::WaypointValidity::Response &resp)
{
  double wpDepth = req.wp.position.z;
  resp.valid = true;
  if (wpDepth > 0)
  {
    resp.valid = false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_validity_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("is_waypoint_valid", isWaypointValid);
  ROS_INFO("Ready to evaluate waypoint.");
  ros::spin();

  return 0;
}
