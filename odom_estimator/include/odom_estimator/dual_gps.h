#ifndef GUARD_KNOQUJTPVNRKUWOW
#define GUARD_KNOQUJTPVNRKUWOW

#include <boost/foreach.hpp>

#include <rawgps_common/Measurements.h>

#include <odom_estimator/manifold.h>
#include <odom_estimator/unscented_transform.h>
#include <odom_estimator/kalman.h>

namespace odom_estimator {
namespace dual_gps {


ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(State,
  (ros::Time, time)
  (std::vector<int>, gps_prn) // XXX assert size == gps_bias.rows() somehow
,
  (Vec<3>, relpos_ecef)
  (Vec<3>, relvel_ecef)
  (Vec<Dynamic>, gps_bias)
)
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()


class StateUpdater : public UnscentedTransformDistributionFunction<State, State, Vec<3>> {
  ros::Time new_time;
  
  GaussianDistribution<Vec<3>> get_extra_distribution() const {
    Vec<3> stddev; stddev << 1, 1, 1;
    return GaussianDistribution<Vec<3>>(
      Vec<3>::Zero(),
      stddev.cwiseProduct(stddev).asDiagonal());
  }
  State apply(State const &state, Vec<3> const &extra) const {
    double dt = (new_time - state.time).toSec();
    assert(dt >= 0);
    
    Vec<3> relaccel_ecef = extra;
    
    return State(
      new_time,
      state.gps_prn,
      state.relpos_ecef + dt * state.relvel_ecef + dt*dt/2 * relaccel_ecef,
      state.relvel_ecef + dt * relaccel_ecef,
      state.gps_bias);
  }

public:
  StateUpdater(ros::Time new_time) :
    new_time(new_time) {
  }
};

class ErrorObserver : public UnscentedTransformDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> > {
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    assert(false);
  }
  
  Vec<Dynamic> apply(State const &state, Vec<Dynamic> const &noise) const {
    assert(false);
  }

public:
  ErrorObserver(rawgps_common::Measurements const & a,
                rawgps_common::Measurements const & b) {
  }
};


GaussianDistribution<State>
update_gps_bias_set(GaussianDistribution<State> const &state,
                    std::vector<rawgps_common::Satellite> const &sats2) {
  std::vector<rawgps_common::Satellite> sats;
  std::vector<int> prns, new_prns;
  for(unsigned int i = 0; i < sats2.size(); i++) {
    rawgps_common::Satellite const &sat = sats2[i];
    if(isnan(sat.carrier_distance)) continue;
    sats.push_back(sat);
    prns.push_back(sat.prn);
    if(std::find(state.mean.gps_prn.begin(), state.mean.gps_prn.end(), sat.prn) == state.mean.gps_prn.end()) {
      new_prns.push_back(sat.prn);
    }
  }
  
  return EasyDistributionFunction<State, State, Vec<Dynamic> >(
    [&sats, &prns, &new_prns](State const &state, Vec<Dynamic> const &noise) {
      Vec<3> pos = state.getPosECEF();
      
      std::vector<double> estimated_carrier_distance_biases;
      BOOST_FOREACH(rawgps_common::Satellite const &sat, sats) {
        if(isnan(sat.carrier_distance)) continue;
        unsigned int index = std::find(state.gps_prn.begin(), state.gps_prn.end(), sat.prn) - state.gps_prn.begin();
        if(index == state.gps_prn.size()) continue;
        double bias = state.gps_prn[index];
        estimated_carrier_distance_biases.push_back((sat.carrier_distance - bias) - ((pos - point2vec(sat.position)).norm() + (sat.T_iono + sat.T_tropo)*c));
      }
      double estimated_carrier_distance_bias =
        estimated_carrier_distance_biases.size() == 0 ? 0 :
          std::accumulate(estimated_carrier_distance_biases.begin(),
                          estimated_carrier_distance_biases.end(),
                          0.)/estimated_carrier_distance_biases.size();
      
      State new_state = state;
      new_state.gps_prn = prns;
      new_state.gps_bias = Vec<Dynamic>(prns.size());
      for(unsigned int i = 0; i < prns.size(); i++) {
        int prn = prns[i];
        rawgps_common::Satellite const &sat = sats[i];
        unsigned int new_prn_index = std::find(new_prns.begin(), new_prns.end(), prn) - new_prns.begin();
        if(new_prn_index != new_prns.size()) {
          new_state.gps_bias(i) = sat.carrier_distance - estimated_carrier_distance_bias - ((pos - point2vec(sat.position)).norm() + (sat.T_iono + sat.T_tropo)*c) + noise(new_prn_index);
        } else {
          unsigned int old_prn_index = std::find(state.gps_prn.begin(), state.gps_prn.end(), prn) - state.gps_prn.begin();
          assert(old_prn_index != state.gps_prn.size());
          new_state.gps_bias(i) = state.gps_bias(old_prn_index);
        }
      }
      return new_state;
    },
    GaussianDistribution<Vec<Dynamic> >(
      Vec<Dynamic>::Zero(new_prns.size()),
      pow(1e3, 2)*Vec<Dynamic>::Ones(new_prns.size()).asDiagonal())
  )(state);
}

GaussianDistribution<State>
center_gps_biases(GaussianDistribution<State> const &state) {
  return kalman_update(
    EasyDistributionFunction<State, Vec<1>, Vec<1> >(
      [](State const &state, Vec<1> const &measurement_noise) {
        return scalar_matrix(state.gps_bias.sum() / state.gps_prn.size()
          - measurement_noise(0));
      },
      GaussianDistribution<Vec<1> >(Vec<1>::Zero(), scalar_matrix(pow(1e3, 2)))),
    state);
}


struct Worker {
  boost::optional<GaussianDistribution<State>> opt_state_dist;
  
  Worker() {
  }
  
  void handle_gps_pair(rawgps_common::Measurements const &a, rawgps_common::Measurements const &b) {
    assert(a.sync == b.sync);
    
    if(!opt_state_dist) {
      Vec<6> stddev; stddev << 10,10,10, 1,1,1;
      opt_state_dist = GaussianDistribution<State>(
        State(a.header.stamp, std::vector<int>{}, Vec<3>::Zero(), Vec<3>::Zero(), Vec<Dynamic>()),
        stddev.cwiseProduct(stddev).asDiagonal());
    } else {
      opt_state_dist = StateUpdater(a.header.stamp)(*opt_state_dist);
    }
    
    opt_state_dist = update_gps_bias_set(*opt_state_dist, a.satellites);
    
    opt_state_dist = kalman_update(ErrorObserver(a, b), *opt_state_dist);
    
    opt_state_dist = center_gps_biases(*opt_state_dist);
  }
};
  


}
}

#endif
