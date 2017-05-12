#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

class WaypointValidity
{
private:
  ros::NodeHandle *nh_;
  nav_msgs::OccupancyGridConstPtr ogrid_map_;
  ros::Subscriber sub_;

  bool check_if_hit(cv::Point center, cv::Size sub_size);

public:
  WaypointValidity(ros::NodeHandle &nh);

  void ogrid_callback(const nav_msgs::OccupancyGridConstPtr &ogrid_map);

  void is_waypoint_valid(const geometry_msgs::Pose &waypoint, bool &valid, int &error);
};