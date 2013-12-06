#ifndef GUARD_MWTDQGRCKCNYRUAC
#define GUARD_MWTDQGRCKCNYRUAC

#include <rawgps_common/Measurements.h>

#include <boost/foreach.hpp>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <unsupported/Eigen/AutoDiff>

#include <odom_estimator/util.h>
#include <odom_estimator/state.h>
#include <odom_estimator/kalman.h>

namespace odom_estimator {



static const double c = 299792458;

typedef ManifoldPair<Vec<3>, Vec<3> > Fix;

class E : public IDistributionFunction<Fix, Vec<Dynamic>, Vec<Dynamic> > {
  rawgps_common::Measurements const &msg;
  
public:
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    Vec<Dynamic> mean = Vec<Dynamic>::Zero(2 + 2*msg.satellites.size());
    
    Vec<Dynamic> stddev(2 + 2*msg.satellites.size());
    stddev(0) = 1e-3;
    stddev(1) = 100;
    for(unsigned int i = 0; i < msg.satellites.size(); i++) {
      rawgps_common::Satellite const &sat = msg.satellites[i];
      
      double cn0 = pow(10, sat.cn0/10.);
      
      stddev(2 + i) = 1;
      stddev(2 + msg.satellites.size() + i) =
        20.5564182949/pow(cn0, 0.691171572741);
    }
    
    return GaussianDistribution<Vec<Dynamic> >(mean,
      stddev.cwiseProduct(stddev).asDiagonal());
  }
  
  Vec<Dynamic> apply(Fix const &fix, Vec<Dynamic> const &noise) const {
    std::vector<rawgps_common::Satellite> const &sats = msg.satellites;
    
    Vec<3> pos = fix.first;
    Vec<3> vel = fix.second;
    
    double t = noise(0);
    double skew = noise(1);
    
    Vec<3> eci_pos = inertial_from_ecef(t, pos);
    Vec<3> eci_vel = inertial_vel_from_ecef_vel(t, vel, eci_pos);
    
    Vec<Dynamic> res = Vec<Dynamic>(2*sats.size());
    for(unsigned int i = 0; i < sats.size(); i++) {
      rawgps_common::Satellite const &sat = sats[i];
      Vec<3> sat_eci_pos = inertial_from_ecef(sat.time, point2vec(sat.position));
      Vec<3> sat_eci_vel = inertial_vel_from_ecef_vel(sat.time, xyz2vec(sat.velocity), sat_eci_pos);
      
      res(i) = (eci_pos - sat_eci_pos).norm() - (t - sat.time)*c + noise(2 + i);
      res(i + sats.size()) = (sat_eci_pos - eci_pos).normalized().dot(eci_vel - sat_eci_vel)
        - (skew + sat.doppler_velocity) + noise(2 + sats.size() + i);
    }
    return res;
  }
  
  E(rawgps_common::Measurements const &msg) :
    msg(msg) {
  }
};

GaussianDistribution<Fix> get_fix(rawgps_common::Measurements const &msg) {
  Fix trial(Vec<3>::Zero(), Vec<3>::Zero());
  
  for(int i = 0; i < 10; i++) {
    Vec<6> prior_stddev = (Vec<6>() << 1e3, 1e3, 1e3, 1e3, 1e3, 1e3).finished();
    GaussianDistribution<Fix> res =
      kalman_update(
        E(msg),
        GaussianDistribution<Fix>(
          trial,
          prior_stddev.cwiseProduct(prior_stddev).asDiagonal()));
    
    if(i == 9) return res;
    
    trial = res.mean;
  }
  
  assert(false);
}

class GPSErrorObserver : public IDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> > {
  E inner;
  Vec<3> const gps_pos;
  Vec<3> const last_gyro;
  
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    return inner.get_extra_distribution();
  }
  
  Vec<Dynamic> apply(State const &state, Vec<Dynamic> const &noise) const {
    return inner.apply(
      Fix(
        state.getPosECEF(gps_pos),
        state.getVelECEF(gps_pos, last_gyro)),
      noise);
  }

public:
  GPSErrorObserver(rawgps_common::Measurements const &msg,
                   Vec<3> const &gps_pos,
                   Vec<3> const &last_gyro) :
    inner(msg), gps_pos(gps_pos), last_gyro(last_gyro) {
  }
};



}

#endif
