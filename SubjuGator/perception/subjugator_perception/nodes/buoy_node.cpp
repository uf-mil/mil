#include <subjugator_perception/buoy.hpp>

#include "ros/ros.h"

#ifdef VISUALIZE
#warning(Compiling with visualization enabled)
#else
#warning(Compiling with NO visualization)
#endif

#ifdef SEGMENTATION_DEBUG
#warning(Compiling with segmentation debugging info enabled)
#else
#warning(Compiling with NO segmentation debugging info)
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_buoy");
  ROS_INFO("Initializing node /pcl_buoy");
  boost::shared_ptr<SubjuGatorBuoyDetector> subjugator_buoys(new SubjuGatorBuoyDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
