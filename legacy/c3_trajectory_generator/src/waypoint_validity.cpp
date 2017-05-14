#include "waypoint_validity.hpp"

// Point and sub_size must be relative to ogrid, IE: meters * 1/OGRID_RESOLUTION + offset
bool WaypointValidity::check_if_hit(cv::Point center, cv::Size sub_size)
{
  for (int x = center.x - sub_size.width / 2; x < center.x + sub_size.width / 2; ++x)
  {
    for (int y = center.y - sub_size.height / 2; y < center.y + sub_size.height / 2; ++y)
    {
      if (ogrid_map_->data.at(x + y * ogrid_map_->info.width) == 99)
      {
        return true;
      }
    }
  }
  return false;
}

void WaypointValidity::ogrid_callback(const nav_msgs::OccupancyGridConstPtr &ogrid_map)
{
  this->ogrid_map_ = ogrid_map;
}

// Convert waypoint to be relative to ogrid, then do a series of checks (unknown, occupied, or above water).
// Returns a bool that represents if the move is safe, and an error
std::pair<bool, WAYPOINT_ERROR_TYPE> WaypointValidity::is_waypoint_valid(const geometry_msgs::Pose &waypoint)
{

  if (waypoint.position.z > 0.2)
  {
    return std::make_pair(false, ABOVE_WATER);
  }

  if(!this->ogrid_map_)
  {
    ROS_ERROR("WaypointValidity - Did not recieve any ogrid");
    return std::make_pair(true, NO_OGRID);
  }

  cv::Point where_sub = cv::Point(waypoint.position.x / ogrid_map_->info.resolution + ogrid_map_->info.width / 2,
                                  waypoint.position.y / ogrid_map_->info.resolution + ogrid_map_->info.height / 2);

  if (ogrid_map_->data.at(where_sub.x + where_sub.y * ogrid_map_->info.width) == 50)
  {
    return std::make_pair(false, UNKNOWN);
  }

  if (check_if_hit(where_sub, cv::Size(5, 5)))
  {
    return std::make_pair(false, OCCUPIED);
  }

  return std::make_pair(true, UNOCCUPIED);
}

WaypointValidity::WaypointValidity(ros::NodeHandle &nh)
{
  nh_ = &nh;
  sub_ = nh_->subscribe<nav_msgs::OccupancyGrid>("/ogrid_pointcloud/ogrid", 1,
                                                 boost::bind(&WaypointValidity::ogrid_callback, this, _1));
}