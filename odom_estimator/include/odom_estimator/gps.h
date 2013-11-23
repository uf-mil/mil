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
static const double w_E = 0.729211510e-4; // from glonass

Vec<3> inertial_from_ecef(double t, Vec<3> pos) {
  double th = w_E * t;
  return Vec<3>(
    pos(0)*cos(th) - pos(1)*sin(th),
    pos(0)*sin(th) + pos(1)*cos(th),
    pos(2));
}
Vec<3> inertial_vel_from_ecef_vel(double t, Vec<3> ecef_vel, Vec<3> inertial_point) {
  double th = w_E * t;
  return Vec<3>(
      ecef_vel(0)*cos(th) - ecef_vel(1)*sin(th) - w_E * inertial_point[1],
      ecef_vel(0)*sin(th) + ecef_vel(1)*cos(th) + w_E * inertial_point[0],
      ecef_vel(2));
}

typedef ManifoldPair<Vec<3>, Vec<3> > Fix;

class E : public IDistributionFunction<Fix, Vec<Dynamic>, Vec<Dynamic> > {
  rawgps_common::Measurements const &msg;
  
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
  
public:
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
