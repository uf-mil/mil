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
  (Vec<3>, relpos_enu)
  (Vec<3>, relvel_enu)
  (Vec<Dynamic>, gps_bias)
)
  double getGPSBias(int prn) const {
    unsigned int index = std::find(gps_prn.begin(), gps_prn.end(), prn) - gps_prn.begin();
    assert(index != gps_prn.size());
    return gps_bias(index);
  }
  boost::optional<double> maybeGetGPSBias(int prn) const {
    unsigned int index = std::find(gps_prn.begin(), gps_prn.end(), prn) - gps_prn.begin();
    if(index == gps_prn.size()) return boost::none;
    return gps_bias(index);
  }
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
    assert(dt < 1);
    
    Vec<3> relaccel_enu = extra;
    
    return State(
      new_time,
      state.gps_prn,
      state.relpos_enu + dt * state.relvel_enu + dt*dt/2 * relaccel_enu,
      state.relvel_enu + dt * relaccel_enu,
      state.gps_bias);
  }

public:
  StateUpdater(ros::Time new_time) :
    new_time(new_time) {
  }
};

std::set<int> get_good_prns(rawgps_common::Measurements const & msg) {
  std::set<int> result;
  BOOST_FOREACH(rawgps_common::Satellite const & sat, msg.satellites) {
    if(!std::isnan(sat.carrier_distance)) {
      result.insert(sat.prn);
    }
  }
  return result;
}
std::set<int> get_good_prns(rawgps_common::Measurements const & a,
                            rawgps_common::Measurements const & b) {
  std::set<int> a_prns = get_good_prns(a);
  std::set<int> b_prns = get_good_prns(b);
  
  std::set<int> result;
  set_intersection(a_prns.begin(), a_prns.end(), b_prns.begin(), b_prns.end(),
                   std::inserter(result, result.begin()));
  return result;
}

rawgps_common::Satellite const & get_sat(rawgps_common::Measurements const & msg,
                                         int prn) {
  BOOST_FOREACH(rawgps_common::Satellite const & sat, msg.satellites) {
    if(sat.prn == prn) {
      return sat;
    }
  }
  assert(false);
}

class ErrorObserver : public UnscentedTransformDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> > {
  rawgps_common::Measurements a;
  rawgps_common::Measurements b;
  
  GaussianDistribution<Vec<Dynamic> > get_extra_distribution() const {
    std::set<int> good_prns = get_good_prns(a, b);
    int N = 1 + good_prns.size();
    return GaussianDistribution<Vec<Dynamic>>(
      Vec<Dynamic>::Zero(N),
      Vec<Dynamic>::Ones(N).asDiagonal());
  }
  
  Vec<Dynamic> apply(State const &state, Vec<Dynamic> const &noise) const {
    std::set<int> good_prns = get_good_prns(a, b);
    if(good_prns.size() == 0) return Vec<Dynamic>();
    
    double carrier_distance_bias_sum = 0;
    unsigned int count = 0;
    BOOST_FOREACH(int prn, good_prns) {
      rawgps_common::Satellite const & a_sat = get_sat(a, prn);
      rawgps_common::Satellite const & b_sat = get_sat(b, prn);
      
      double carrier_distance_difference = a_sat.carrier_distance - b_sat.carrier_distance;
      
      carrier_distance_bias_sum += carrier_distance_difference - (xyz2vec(a_sat.direction_enu).dot(state.relpos_enu) + state.getGPSBias(prn));
      count++;
    }
    assert(count);
    double carrier_distance_bias = carrier_distance_bias_sum/count + 10*noise(0);
    
    Vec<Dynamic> res(good_prns.size());
    { int i = 0; BOOST_FOREACH(int prn, good_prns) {
      rawgps_common::Satellite const & a_sat = get_sat(a, prn);
      rawgps_common::Satellite const & b_sat = get_sat(b, prn);
      
      double carrier_distance_difference = a_sat.carrier_distance - b_sat.carrier_distance;
      
      double predicted_carrier_distance_difference = xyz2vec(a_sat.direction_enu).dot(state.relpos_enu) + state.getGPSBias(prn) + carrier_distance_bias + 0.05*noise(1+i);
      
      res(i) = carrier_distance_difference - predicted_carrier_distance_difference;
    i++; } }
    return res;
  }

public:
  ErrorObserver(rawgps_common::Measurements const & a,
                rawgps_common::Measurements const & b) :
    a(a), b(b) {
  }
};


GaussianDistribution<State>
update_gps_bias_set(GaussianDistribution<State> const &state,
                    rawgps_common::Measurements const & a,
                    rawgps_common::Measurements const & b) {
  std::set<int> good_prns = get_good_prns(a, b);
  
  return EasyDistributionFunction<State, State, Vec<Dynamic> >(
    [&a, &b, &good_prns](State const &state, Vec<Dynamic> const &noise) {
      double carrier_distance_bias_sum = 0;
      unsigned int count = 0;
      BOOST_FOREACH(int prn, good_prns) {
        if(!state.maybeGetGPSBias(prn)) continue;
        
        rawgps_common::Satellite const & a_sat = get_sat(a, prn);
        rawgps_common::Satellite const & b_sat = get_sat(b, prn);
        
        double carrier_distance_difference = a_sat.carrier_distance - b_sat.carrier_distance;
        
        carrier_distance_bias_sum += carrier_distance_difference - (xyz2vec(a_sat.direction_enu).dot(state.relpos_enu) + state.getGPSBias(prn));
        count++;
      }
      double carrier_distance_bias = (count ? carrier_distance_bias_sum/count : 0) + 10*noise(0);
      
      State new_state = state;
      new_state.gps_prn = std::vector<int>(good_prns.begin(), good_prns.end());
      new_state.gps_bias = Vec<Dynamic>(good_prns.size());
      { int i = 0; BOOST_FOREACH(int prn, good_prns) {
        boost::optional<double> old_bias = state.maybeGetGPSBias(prn);
        if(old_bias) {
          new_state.gps_bias(i) = *old_bias;
        } else {
          std::cout << prn << " joined" << std::endl;
          rawgps_common::Satellite const & a_sat = get_sat(a, prn);
          rawgps_common::Satellite const & b_sat = get_sat(b, prn);
          
          double carrier_distance_difference = a_sat.carrier_distance - b_sat.carrier_distance;
          
          new_state.gps_bias(i) = carrier_distance_difference - (xyz2vec(a_sat.direction_enu).dot(state.relpos_enu) + carrier_distance_bias) + 1*noise(1+i);
        }
      i++; } }
      return new_state;
    },
    GaussianDistribution<Vec<Dynamic> >(
      Vec<Dynamic>::Zero(1+good_prns.size()),
      Vec<Dynamic>::Ones(1+good_prns.size()).asDiagonal())
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
      GaussianDistribution<Vec<1> >(Vec<1>::Zero(), scalar_matrix(pow(1e2, 2)))),
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
    
    std::cout << "a " << opt_state_dist->mean.relpos_enu.transpose() << std::endl;
    
    opt_state_dist = update_gps_bias_set(*opt_state_dist, a, b);
    
    std::cout << "b " << opt_state_dist->mean.relpos_enu.transpose() << std::endl;
    
    opt_state_dist = kalman_update(ErrorObserver(a, b), *opt_state_dist);
    std::cout << "c " << opt_state_dist->mean.relpos_enu.transpose() << std::endl;
    
    //opt_state_dist = center_gps_biases(*opt_state_dist);
    std::cout << "d " << opt_state_dist->mean.relpos_enu.transpose() << std::endl;
    
    opt_state_dist = kalman_update(
      EasyDistributionFunction<State, Vec<1>, Vec<1> >(
        [](State const &state, Vec<1> const &measurement_noise) {
          return scalar_matrix(state.relpos_enu.dot(state.relvel_enu)
            - .1*measurement_noise(0));
        },
        GaussianDistribution<Vec<1> >(Vec<1>::Zero(), scalar_matrix(1))),
      *opt_state_dist);
    std::cout << "e " << opt_state_dist->mean.relpos_enu.transpose() << std::endl;
  }
};
  


}
}

#endif
