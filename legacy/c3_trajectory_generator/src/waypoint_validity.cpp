#include "waypoint_validity.hpp"

// Point and sub_size must be relative to ogrid (IE, in ogrid-cell units)
bool WaypointValidity::check_if_hit(cv::Point center, cv::Size sub_size)
{
  for (int x = center.x - sub_size.width / 2; x < center.x + sub_size.width / 2; ++x)
  {
    for (int y = center.y - sub_size.height / 2; y < center.y + sub_size.height / 2; ++y)
    {
      try
      {
        if (ogrid_map_->data.at(x + y * ogrid_map_->info.width) == (uchar)WAYPOINT_ERROR_TYPE::OCCUPIED)
        {
          return true;
        }
      }
      catch (std::out_of_range &e)
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
std::pair<bool, WAYPOINT_ERROR_TYPE> WaypointValidity::is_waypoint_valid(const geometry_msgs::Pose &waypoint,
                                                                         bool do_waypoint_validation)
{
  if (!do_waypoint_validation)
    return std::make_pair(true, WAYPOINT_ERROR_TYPE::NOT_CHECKED);
  if (waypoint.position.z > 0.2)
  {
    return std::make_pair(false, WAYPOINT_ERROR_TYPE::ABOVE_WATER);
  }

  if (!this->ogrid_map_)
  {
    return std::make_pair(false, WAYPOINT_ERROR_TYPE::NO_OGRID);
  }

  cv::Point where_sub = cv::Point(waypoint.position.x / ogrid_map_->info.resolution + ogrid_map_->info.width / 2,
                                  waypoint.position.y / ogrid_map_->info.resolution + ogrid_map_->info.height / 2);

  // Area we want to check around the sub
  int sub_x = 1.5 / ogrid_map_->info.resolution;
  int sub_y = 1.5 / ogrid_map_->info.resolution;
  if (check_if_hit(where_sub, cv::Size(sub_x, sub_y)))
  {
    return std::make_pair(false, WAYPOINT_ERROR_TYPE::OCCUPIED);
  }
  try
  {
    if (ogrid_map_->data.at(where_sub.x + where_sub.y * ogrid_map_->info.width) == (uchar)WAYPOINT_ERROR_TYPE::UNKNOWN)
    {
      return std::make_pair(false, WAYPOINT_ERROR_TYPE::UNKNOWN);
    }
  }
  catch (std::out_of_range &e)
  {
    return std::make_pair(false, WAYPOINT_ERROR_TYPE::OCCUPIED);
  }

  return std::make_pair(true, WAYPOINT_ERROR_TYPE::UNOCCUPIED);
}

WaypointValidity::WaypointValidity(ros::NodeHandle &nh)
{
  nh_ = &nh;
  sub_ = nh_->subscribe<nav_msgs::OccupancyGrid>("/ogrid_pointcloud/ogrid", 1,
                                                 boost::bind(&WaypointValidity::ogrid_callback, this, _1));
}