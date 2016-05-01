#include <sub8_pcl/torpedo_board.hpp>
#include <ros/ros.h>

// #define VISUALIZE
// #define SEGMENTATION_DEBUG

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_perception");
  ROS_INFO("Initializing node /pcl_perception");
  Sub8TorpedoBoardDetector torpedo_board_detector(0.5);
  ros::spin();
}
