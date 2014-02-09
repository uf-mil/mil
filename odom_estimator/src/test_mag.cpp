#include <ros/ros.h>

#include "odom_estimator/magnetic.h"

using namespace odom_estimator;

int main(int argc, char** argv) {
  magnetic::MagneticModel mm(ros::package::getPath("odom_estimator") + "/data/WMM.COF");
  
  double t = 1262322000; // 2010.0
  std::cout << ecef_from_inertial(t, mm.getField(inertial_from_ecef(t, Vec<3>(-555582.43540501, -962297.00591432, -6259542.96102869)), t)).transpose() << std::endl;
  std::cout << Vec<3>(6197.3580730669782, -20720.516899810533, -51613.691745554344).transpose() << std::endl;
  
  return 0;
}
