#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>
#include "ContourMethod/FrameProc.h"
#include "ContourMethod/ShapeFind.h"
#include "DebugWindow.h"
#include "opencv2/opencv.hpp"
#include <navigator_msgs/DockShapes.h>
#include <navigator_msgs/GetDockShape.h>
#include <navigator_msgs/GetDockShapes.h>
#include "std_srvs/SetBool.h"

using namespace std;
class TrackedShape
{
  private:
    unsigned int count;
    static unsigned int MIN_COUNT;
    static double MAX_DISTANCE_GAP;
    static ros::Duration MAX_TIME_GAP;
    navigator_msgs::DockShape latest;
    static double centerDistance(navigator_msgs::DockShape& a, navigator_msgs::DockShape& b);
    bool tooOld(navigator_msgs::DockShape& s);
  public:
    TrackedShape();
    TrackedShape(navigator_msgs::DockShape& s);
    bool update(navigator_msgs::DockShape& s);
    bool isReady();
    bool isStale();
    navigator_msgs::DockShape get();
    static void init(ros::NodeHandle& nh);
};

class ShapeTracker
{
  private:
    vector<TrackedShape> shapes;
  public:
    void ShapeTracker();
    void init(ros::NodeHandle& nh);
    void addShapes(navigator_msgs::DockShapes& newShapes);
};
