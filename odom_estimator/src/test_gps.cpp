#include <ros/ros.h>

#include "odom_estimator/gps.h"

using namespace odom_estimator;

void got_gps(rawgps_common::MeasurementsConstPtr const &msgp) {
  rawgps_common::Measurements const msg = *msgp;
  
  GaussianDistribution<Fix> fixdist = get_fix(msg);
  
  std::cout << "pos: " << fixdist.mean.first.transpose() << std::endl;
  std::cout << "vel: " << fixdist.mean.second.transpose() << std::endl;
  std::cout << "cov: " << fixdist.cov << std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_gps");
  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("gps", 1, got_gps);
  
  ros::spin();
  
  return 0;
}
