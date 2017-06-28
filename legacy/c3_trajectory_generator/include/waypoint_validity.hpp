#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

enum class WAYPOINT_ERROR_TYPE
{
  OCCUPIED = 99,
  UNKNOWN = 50,
  UNOCCUPIED = 0,
  ABOVE_WATER = 1,

  NO_OGRID = 100,
  NOT_CHECKED = 2,
  OCCUPIED_TRAJECTORY = 98
};

enum class OGRID_COLOR
{
  ORANGE = 200,
  RED = 130,
  GREEN = 120
};

class WaypointValidity
{
private:
  ros::NodeHandle *nh_;
  nav_msgs::OccupancyGridConstPtr ogrid_map_;
  ros::Subscriber sub_;

  ros::Publisher pub_sub_ogrid_;
  ros::Publisher pub_waypoint_ogrid_;
  double sub_ogrid_size_;

  // Usage: Given a size and point, relative to ogrid, will check if there is an occupied grid
  bool check_if_hit(cv::Point center, cv::Size sub_size);

public:
  WaypointValidity(ros::NodeHandle &nh);

  // Usage: Store the reference to the previous ogrid in publisher
  void ogrid_callback(const nav_msgs::OccupancyGridConstPtr &ogrid_map);

  void pub_size_ogrid(const geometry_msgs::Pose &waypoint, int d = 0);

  // Usage: Given a waypoint or trajectory, check what it will hit on the ogrid.
  std::pair<bool, WAYPOINT_ERROR_TYPE> is_waypoint_valid(const geometry_msgs::Pose &waypoint,
                                                         bool do_waypoint_validation);
};