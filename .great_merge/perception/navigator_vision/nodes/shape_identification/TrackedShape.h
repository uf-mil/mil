#include <ros/ros.h>
#include <vector>
#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/GetDockShape.h>
#include <navigator_msgs/GetDockShapes.h>
#include <string>

class TrackedShape
{
  private:
    int count;
    static int MIN_COUNT;
    static double MAX_DISTANCE;
    static ros::Duration MAX_TIME_GAP;
    navigator_msgs::DockShape latest;
    static double centerDistance(navigator_msgs::DockShape& a, navigator_msgs::DockShape& b);
    bool tooOld(navigator_msgs::DockShape& s);
  public:
    TrackedShape();
    TrackedShape(navigator_msgs::DockShape& s);
    bool sameType(std::string& color, std::string & shape);
    bool update(navigator_msgs::DockShape& s);
    bool isReady();
    bool isStale();
    navigator_msgs::DockShape get();
    static void init(ros::NodeHandle& nh);
};
