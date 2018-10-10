#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "point_cloud_object_detector");
  ros::NodeHandle nh(ros::this_node::getName());
  pcodar::pcodar_controller c(nh);
  c.initialize();
  ros::spin();
  return 0;
}
