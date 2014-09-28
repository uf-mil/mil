#ifndef GUARD_MWTDQGRCKCNYRUAC
#define GUARD_MWTDQGRCKCNYRUAC

#include <rawgps_common/Measurements.h>

#include <boost/foreach.hpp>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <unsupported/Eigen/AutoDiff>

#include "odom_estimator/util.h"
#include "odom_estimator/state.h"
#include "odom_estimator/kalman.h"

namespace odom_estimator {



static const double c = 299792458;

typedef ManifoldPair<Vec<3>, Vec<3> > Fix;
typedef ManifoldPair<Fix, Vec<Dynamic>> FixWithBias;

class E : public UnscentedTransformDistributionFunction<FixWithBias, Vec<Dynamic>, Vec<Dynamic> > {
  rawgps_common::Measurements const &msg;
  std::vector<int> gps_prn;
  
public:
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    Vec<Dynamic> mean = Vec<Dynamic>::Zero(2 + 2*msg.satellites.size());
    
    Vec<Dynamic> stddev(2 + 2*msg.satellites.size());
    stddev(0) = 100;
    stddev(1) = 100;
    for(unsigned int i = 0; i < msg.satellites.size(); i++) {
      rawgps_common::Satellite const &sat = msg.satellites[i];
      
      double cn0 = pow(10, sat.cn0/10.);
      
      stddev(2 + i) = 5;
      stddev(2 + msg.satellites.size() + i) =
        20.5564182949/pow(cn0, 0.691171572741);
    }
    
    return GaussianDistribution<Vec<Dynamic> >(mean,
      stddev.cwiseProduct(stddev).asDiagonal());
  }
  
  Vec<Dynamic> apply(FixWithBias const &fixwithbias, Vec<Dynamic> const &noise) const {
    Fix const &fix = fixwithbias.first;
    std::vector<rawgps_common::Satellite> const &sats = msg.satellites;
    
    Vec<3> pos = fix.first;
    Vec<3> vel = fix.second;
    
    // estimate t of reception, ignoring earth's rotation (rotation's effect
    // only adds about +/-120 ns, which shouldn't matter)
    std::vector<double> estimated_ts;
    std::vector<double> estimated_skews;
    for(unsigned int i = 0; i < sats.size(); i++) {
      rawgps_common::Satellite const &sat = sats[i];
      estimated_ts.push_back((sat.time + sat.T_iono + sat.T_tropo) + (pos - point2vec(sat.position)).norm()/c);
      estimated_skews.push_back((point2vec(sat.position) - pos).normalized().dot(vel - xyz2vec(sat.velocity)) - sat.doppler_velocity);
    }
    double t = noise(0)/c + std::accumulate(estimated_ts.begin(), estimated_ts.end(), 0.)/estimated_ts.size();
    double skew = noise(1) + std::accumulate(estimated_skews.begin(), estimated_skews.end(), 0.)/estimated_skews.size();
    
    Vec<3> eci_pos = inertial_from_ecef(t, pos);
    Vec<3> eci_vel = inertial_vel_from_ecef_vel(t, vel, eci_pos);
    
    Vec<Dynamic> res = Vec<Dynamic>(2*sats.size());
    for(unsigned int i = 0; i < sats.size(); i++) {
      rawgps_common::Satellite const &sat = sats[i];
      Vec<3> sat_eci_pos = inertial_from_ecef(sat.time, point2vec(sat.position));
      Vec<3> sat_eci_vel = inertial_vel_from_ecef_vel(sat.time, xyz2vec(sat.velocity), sat_eci_pos);
      
      unsigned int index = std::find(gps_prn.begin(), gps_prn.end(), sat.prn) - gps_prn.begin();
      assert(index != gps_prn.size());
      double bias = fixwithbias.second[index];
      res(i) = (eci_pos - sat_eci_pos).norm() - (t - (sat.time + sat.T_iono + sat.T_tropo))*c + noise(2 + i);
      res(i + sats.size()) = (sat_eci_pos - eci_pos).normalized().dot(eci_vel - sat_eci_vel)
        - (skew + sat.doppler_velocity) + noise(2 + sats.size() + i);
    }
    return res;
  }
  
  E(rawgps_common::Measurements const &msg, std::vector<int> const &gps_prn) :
    msg(msg), gps_prn(gps_prn) {
  }
};

GaussianDistribution<Fix> get_fix(rawgps_common::Measurements const &msg) {
  std::vector<int> prns;
  for(unsigned int i = 0; i < msg.satellites.size(); i++) {
    prns.push_back(msg.satellites[i].prn);
  }
  
  Fix trial(Vec<3>::Zero(), Vec<3>::Zero());
  
  for(int i = 0; i < 10; i++) {
    Vec<6> prior_stddev = (Vec<6>() << 1e3, 1e3, 1e3, 1e3, 1e3, 1e3).finished();
    GaussianDistribution<FixWithBias> res =
      kalman_update(
        E(msg, prns),
        GaussianDistribution<FixWithBias>(
          FixWithBias(
            trial,
            Vec<Dynamic>::Zero(prns.size())),
          joinDiagonally(
            prior_stddev.cwiseProduct(prior_stddev).asDiagonal(),
            Vec<Dynamic>::Ones(prns.size()).asDiagonal())));
    
    if(i == 9) {
      return EasyDistributionFunction<FixWithBias, Fix, Vec<0> >(
        [](FixWithBias const &fixwithbias, Vec<0> const &) {
          return fixwithbias.first;
        },
        GaussianDistribution<Vec<0> >(Vec<0>(), SqMat<0>())
      )(res);
    }
    
    trial = res.mean.first;
  }
  
  assert(false);
}

class GPSErrorObserver : public UnscentedTransformDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> > {
  E inner;
  Vec<3> const gps_pos;
  Vec<3> const last_gyro;
  
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    return inner.get_extra_distribution();
  }
  
  Vec<Dynamic> apply(State const &state, Vec<Dynamic> const &noise) const {
    return inner.apply(
      FixWithBias(
        Fix(
          state.getPosECEF(gps_pos),
          state.getVelECEF(gps_pos, last_gyro)),
        state.gps_bias),
      noise);
  }

public:
  GPSErrorObserver(rawgps_common::Measurements const &msg,
                   std::vector<int> gps_prn,
                   Vec<3> const &gps_pos,
                   Vec<3> const &last_gyro) :
    inner(msg, gps_prn), gps_pos(gps_pos), last_gyro(last_gyro) {
  }
};



}

#endif
