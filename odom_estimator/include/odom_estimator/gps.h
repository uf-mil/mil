#ifndef _IDNPEJCVJIKZHSKG_
#define _IDNPEJCVJIKZHSKG_

#include <Eigen/Dense>

namespace odom_estimator {

static inline VectorXd gps_observer(const rawgps_common::Measurements &msg,
                      const Vector3d &gps_pos,
                      const Vector3d &last_gyro,
                      const StateWithMeasurementNoise<Dynamic> &tmp) {
  State const &state = tmp.first;
  Matrix<double, Dynamic, 1> measurement_noise = tmp.second;
  
  VectorXd res(msg.satellites.size());
  Vector3d gps_vel = state.vel + state.orient._transformVector(
    (last_gyro - state.gyro_bias).cross(gps_pos));
  for(unsigned int i = 0; i < msg.satellites.size(); i++) {
    const rawgps_common::Satellite &sat = msg.satellites[i];
    
    double predicted = xyz2vec(sat.direction_enu).dot(gps_vel) +
      measurement_noise(i) + measurement_noise(measurement_noise.size()-1);
    res[i] = sat.velocity_plus_drift - predicted;
  }
  return res;
}

}

#endif
