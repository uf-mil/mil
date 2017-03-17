#include "TrackedShape.h"
using namespace std;
int TrackedShape::MIN_COUNT = 20;
double TrackedShape::MAX_DISTANCE = 5;
ros::Duration TrackedShape::MAX_TIME_GAP = ros::Duration(0, 0.5 * 1E9);
TrackedShape::TrackedShape() : count(0)
{
  
}
TrackedShape::TrackedShape(navigator_msgs::DockShape& s) :
  count(1),
  latest(s)
{
  
}
void TrackedShape::init(ros::NodeHandle& nh)
{
  nh.param<int>("tracking/min_count",MIN_COUNT,10);
  nh.param<double>("tracking/max_distance_gap",MAX_DISTANCE,15);
  double seconds;
  nh.param<double>("tracking/max_seen_gap_seconds", seconds, 0.5);
  double partial_secs = fmod(seconds, 1);
  seconds -= partial_secs;
  MAX_TIME_GAP = ros::Duration(seconds, partial_secs * 1E9);
}
double TrackedShape::centerDistance(navigator_msgs::DockShape& a, navigator_msgs::DockShape& b)
{
  // ~printf("A = (%d,%d) B= (%d,%d)\n",a.CenterX,a.CenterY,b.CenterX,b.CenterY);
  return sqrtf( powf(double(a.CenterX)-double(b.CenterX), 2) + powf(double(a.CenterY)-double(b.CenterY),2) );
}
bool TrackedShape::update(navigator_msgs::DockShape& s)
{
  double distance = centerDistance(latest,s);
  if (distance > MAX_DISTANCE)
  {
    //  printf(" %s %s distance too big to %s %s DISTANCE = %f \n",latest.Color.c_str(),latest.Shape.c_str(),s.Color.c_str(),s.Shape.c_str(),distance);
    return false;
  } 
  if (tooOld(s))
  {
    //  printf(" %s %s too old from %s %s\n",latest.Color.c_str(),latest.Shape.c_str(),s.Color.c_str(),s.Shape.c_str());
    return false;
  }
  if (latest.Color != s.Color)
  {
    //What to do if color is different with two nearby
    if (latest.color_confidence > s.color_confidence)
    {
      s.Color = latest.Color;
      s.color_confidence = latest.color_confidence;
    }
  }
  // ~if (latest.Shape != s.Shape)
  // ~{
    // ~if (s.shapeConfidence)
  // ~}
  latest = s;
  count++;
  // ~printf("Updating %s %s with %s %s COUNT=%d Distance=%f \n",latest.Color.c_str(),latest.Shape.c_str(),s.Color.c_str(),s.Shape.c_str(),count,distance);
  return true;
}
bool TrackedShape::tooOld(navigator_msgs::DockShape& s)
{
  return s.header.stamp - latest.header.stamp > MAX_TIME_GAP;
}
bool TrackedShape::isStale()
{
  ros::Time now = ros::Time::now();
  // ~std::cout << "Time Diff: " << (now - latest.header.stamp).toSec() << std::endl;
  return (now - latest.header.stamp) > MAX_TIME_GAP;
}
bool TrackedShape::isReady()
{
  //  printf("Count=%d, TooOld=%d\n", count, isStale());
  return count >= MIN_COUNT && !isStale();
}
navigator_msgs::DockShape TrackedShape::get()
{
  return latest;
}
bool TrackedShape::sameType(std::string& color, std::string& shape)
{
  return (color == navigator_msgs::GetDockShape::Request::ANY ||  color == latest.Color ) && (shape == navigator_msgs::GetDockShape::Request::ANY || shape == latest.Shape);
}
