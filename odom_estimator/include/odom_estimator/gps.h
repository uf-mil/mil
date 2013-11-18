#ifndef _IDNPEJCVJIKZHSKG_
#define _IDNPEJCVJIKZHSKG_

#include <Eigen/Dense>

namespace odom_estimator {

class GPSErrorObserver {
  rawgps_common::Measurements const &msg;
  Vector3d const gps_pos;
  Vector3d const last_gyro;
  typedef Matrix<double, Dynamic, 1> NoiseType;
  typedef Matrix<double, Dynamic, Dynamic> NoiseCovType;
public:
  GPSErrorObserver(rawgps_common::Measurements const &msg,
                   Vector3d const &gps_pos,
                   Vector3d const &last_gyro) :
    msg(msg), gps_pos(gps_pos), last_gyro(last_gyro) {
  }
  
  typedef VectorXd ErrorType;
  
  typedef NoiseType ExtraType;
  typedef NoiseCovType ExtraCovType;
  ExtraType get_extra_mean() const {
    return ExtraType::Zero(msg.satellites.size());
  }
  ExtraCovType get_extra_cov() const {
    double max_cn0 = -std::numeric_limits<double>::infinity();
    BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
      max_cn0 = std::max(max_cn0, satellite.cn0);
    }
    
    VectorXd stddev(msg.satellites.size() + 1);
    BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
      stddev[&satellite - msg.satellites.data()] = .15 / sqrt(pow(10, satellite.cn0/10) / pow(10, max_cn0/10));
    }
    stddev[msg.satellites.size()] = 5000;
    std::cout << "stddev:" << std::endl << stddev << std::endl << std::endl;
    
    return stddev.cwiseProduct(stddev).asDiagonal();
  }
  
  ErrorType observe_error(State const &state, ExtraType const &extra) const {
    NoiseType const &noise = extra;
    
    Vector3d gps_vel = state.vel + state.orient._transformVector(
      (last_gyro - state.gyro_bias).cross(gps_pos));
    
    VectorXd res(msg.satellites.size());
    for(unsigned int i = 0; i < msg.satellites.size(); i++) {
      const rawgps_common::Satellite &sat = msg.satellites[i];
      
      double predicted = xyz2vec(sat.direction_enu).dot(gps_vel) +
        noise(i) + noise(noise.size()-1);
      res[i] = sat.velocity_plus_drift - predicted;
    }
    return res;
  }
};

}

#endif
