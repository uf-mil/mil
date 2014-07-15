#include <ros/ros.h>

#include "odom_estimator/magnetic.h"

using namespace odom_estimator;

int main(int argc, char** argv) {
  magnetic::MagneticModel mm(ros::package::getPath("odom_estimator") + "/data/WMM.COF");
  
  // Test from 9th row of WMM2010 test values
  
  double t = 1262322000 + (365.25*86400) * 2.5; // 2012.5
  Vec<3> pos_ecef(-555582.43540501, -962297.00591432, -6259542.96102869);
  std::cout << 1e9*(enu_from_ecef_mat(pos_ecef) * ecef_from_inertial(t, mm.getField(inertial_from_ecef(t, pos_ecef), t))).transpose() << std::endl;
  std::cout << Vec<3>(15731.8, 5713.6, 53184.3).transpose() << std::endl;
  
  std::cout << mm.getFieldAndJacobian(inertial_from_ecef(t, pos_ecef), t).second << std::endl;
  
  return 0;
}
