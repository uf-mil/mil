#include "Filter.h"

TrackedShape::TrackedShape() : count(0)
{
  
}
TrackedShape::TrackedShape(navigator_msgs::DockShape& s) :
  count(1),
  latest(s)
{
  
}
TrackedShape::init(ros::NodeHandle& nh)
{
  nh.param<int>("tracking/min_count",MIN_COUNT,10);
  nh.param<double>("tracking/max_distance_gap",MAX_DISTANCE,15);
  double seconds;
  nh.param<double>("tracking/max_seen_gap_seconds", seconds, 0.5);
  max_seen_gap_dur = ros::Duration(0, seconds * 1000000000);
}
double TrackedShape::centerDistance(navigator_msgs::DockShape& a, navigator_msgs::DockShape& b)
{
  return sqrtf( pow(a.CenterX-b.CenterX, 2) + pow(a.CenterY,b.CenterY) );
}
bool TrackedShape::update(navigator_msgs::DockShape& s)
{
  if (centerDistance(latest,s) > MAX_DISTANCE) return false;
  if (tooOld(s)) return false;
  latest = s;
  count++;
}
bool TrackedShape::tooOld(navigator_msgs::DockShape& s)
{
  return s.header.stamp - latest.header.stamp > MAX_TIME_GAP;
}
bool TrackedShape::isStale()
{
  ros::Time now = ros::Time::now();
  return (now - buffer.back().header.stamp) > max_seen_gap_dur;
}
bool TrackedShape::isReady()
{
  return count >= MIN_COUNT && !isStale();
}
navigator_msgs::DockShape TrackedShape::get()
{
  return latest;
}
ShapeTracker::ShapeTracker()
{
  
}
ShapeTracker::init(ros::NodeHandle& nh)
{
  
}
ShapeTracker::addShapes(navigator_msgs::DockShapes& newShapes)
{
  eachnew: for (auto shape = newShapes.begin(); shape != newShapes.end(); shape++)
  {
     for (auto tracked = shapes.begin(); tracked != shapes.end(); tracked++)
     {
      if (tracked.update(shape)) continue eachnew;
     }
     shapes.push_back(shape);
  }
}
