#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "sub8_msgs/WaypointValidity.h"
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>
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

class WaypointChecker
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer validity_service_;
  nav_msgs::OccupancyGridConstPtr ogrid_map_;
  ros::Subscriber sub_;
public:

  WaypointChecker() : nh_(ros::this_node::getName())
  {
    sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/ogridgen/ogrid", 1,
                                                  boost::bind(&WaypointChecker::ogrid_callback, this, _1));
    validity_service_ = nh_.advertiseService<sub8_msgs::WaypointValidity::Request, sub8_msgs::WaypointValidity::Response>(
        "is_waypoint_valid", boost::bind(&WaypointChecker::is_waypoint_valid, this, _1, _2));
  }

  // TODO: What the error was
  bool is_waypoint_valid(sub8_msgs::WaypointValidity::Request &req, sub8_msgs::WaypointValidity::Response &resp)
  {
    // Where the /depth at
    double wpDepth = req.wp.position.z;
    resp.valid = true;
    if (wpDepth > 0)
    {
      resp.valid = false;
      return true;
    }

    // Check if point is occupied
    cv::Point where_sub = cv::Point(req.wp.position.x / ogrid_map_->info.resolution + ogrid_map_->info.width / 2,
                                    req.wp.position.y / ogrid_map_->info.resolution + ogrid_map_->info.height / 2);
    if (ogrid_map_->data.at(where_sub.x + where_sub.y * ogrid_map_->info.width) == 99)
    {
      resp.valid = false;
    }

    return true;
  }

  void ogrid_callback(const nav_msgs::OccupancyGridConstPtr &ogrid_map)
  {
    this->ogrid_map_ = ogrid_map;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_validity_node");

  WaypointChecker waypointChecker;
  ROS_INFO("Ready to evaluate waypoint.");
  ros::spin();

  return 0;
}
