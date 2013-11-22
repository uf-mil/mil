#ifndef GUARD_MWTDQGRCKCNYRUAC
#define GUARD_MWTDQGRCKCNYRUAC

#include <rawgps_common/Measurements.h>

#include <boost/foreach.hpp>

#include <Eigen/Dense>

#include <odom_estimator/util.h>
#include <odom_estimator/state.h>

namespace odom_estimator {

class GPSErrorObserver : public IDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> > {
  rawgps_common::Measurements const &msg;
  Vec<3> const gps_pos;
  Vec<3> const last_gyro;
  
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    double max_cn0 = -std::numeric_limits<double>::infinity();
    BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
      max_cn0 = std::max(max_cn0, satellite.cn0);
    }
    
    Vec<Dynamic> stddev(msg.satellites.size() + 1);
    BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
      stddev[&satellite - msg.satellites.data()] = .15 / sqrt(pow(10, satellite.cn0/10) / pow(10, max_cn0/10));
    }
    stddev[msg.satellites.size()] = 5000;
    std::cout << "stddev:" << std::endl << stddev << std::endl << std::endl;
    
    return GaussianDistribution<Vec<Dynamic> >(
      Vec<Dynamic>::Zero(msg.satellites.size()),
      stddev.cwiseProduct(stddev).asDiagonal());
  }
  
  Vec<Dynamic> apply(State const &state, Vec<Dynamic> const &noise) const {
    Vec<3> gps_vel = state.vel + state.orient._transformVector(
      (last_gyro - state.gyro_bias).cross(gps_pos));
    
    Vec<Dynamic> res(msg.satellites.size());
    for(unsigned int i = 0; i < msg.satellites.size(); i++) {
      const rawgps_common::Satellite &sat = msg.satellites[i];
      
      double predicted = xyz2vec(sat.direction_enu).dot(gps_vel) +
        noise(i) + noise(noise.size()-1);
      res(i) = sat.velocity_plus_drift - predicted;
    }
    return res;
  }

public:
  GPSErrorObserver(rawgps_common::Measurements const &msg,
                   Vec<3> const &gps_pos,
                   Vec<3> const &last_gyro) :
    msg(msg), gps_pos(gps_pos), last_gyro(last_gyro) {
  }
};

}

#endif
