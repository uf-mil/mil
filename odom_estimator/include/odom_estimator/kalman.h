#ifndef _VYZZLQVCEVLWEEOA_
#define _VYZZLQVCEVLWEEOA_

#include <odom_estimator/util.h>

namespace odom_estimator {


template<int NN>
using StateWithMeasurementNoise = ManifoldPair<State, Matrix<double, NN, 1> >;

template<typename State>
struct AugmentedState : public State {
  typename State::CovType cov;
  AugmentedState(const State &state, const typename State::CovType &cov) :
    State(state), cov(cov/2 + cov.transpose()/2) {
    assert_none_nan(cov);
  }
  
  template<typename StateUpdator>
  AugmentedState predict(StateUpdator const &stateupdator) const {
    typedef ManifoldPair<State, typename StateUpdator::ExtraType> StateWithExtra;
    
    UnscentedTransform<State, State::RowsAtCompileTime,
      StateWithExtra, StateWithExtra::RowsAtCompileTime> res(
        [&stateupdator](StateWithExtra const &x) {
          return stateupdator.predict(x.first, x.second);
        },
        StateWithExtra(static_cast<const State&>(*this), stateupdator.get_extra_mean()),
        StateWithExtra::build_cov(cov, stateupdator.get_extra_cov()));
    
    return AugmentedState(res.mean, res.cov);
  }
  
  template<typename StateErrorObserver>
  AugmentedState update(StateErrorObserver const &stateerrorobserver) const {
    typedef ManifoldPair<State, typename StateErrorObserver::ExtraType> StateWithExtra;
    typedef typename StateErrorObserver::ErrorType ErrorType;
    State const &state = static_cast<State const &>(*this);
    
    UnscentedTransform<ErrorType, ErrorType::RowsAtCompileTime,
      StateWithExtra, StateWithExtra::RowsAtCompileTime> res(
        [&stateerrorobserver](StateWithExtra const &x) {
          return stateerrorobserver.observe_error(x.first, x.second);
        },
        StateWithExtra(state, stateerrorobserver.get_extra_mean()),
        StateWithExtra::build_cov(cov, stateerrorobserver.get_extra_cov()));
    
    Matrix<double, State::RowsAtCompileTime, ErrorType::RowsAtCompileTime> P_xz =
      res.cross_cov.transpose().template topLeftCorner(state.rows(), res.mean.rows());
    Matrix<double, ErrorType::RowsAtCompileTime, ErrorType::RowsAtCompileTime> P_zz = res.cov;
    
    //Matrix<double, State::RowsAtCompileTime, N> K = P_xz * P_zz.inverse();
    // instead, using matrix solver:
    // K = P_xz P_zz^-1
    // K P_zz = P_xz
    // P_zz' K' = P_xz'
    // K' = solve(P_zz', P_xz')
    // K = solve(P_zz', P_xz')'
    Matrix<double, State::RowsAtCompileTime, ErrorType::RowsAtCompileTime> K =
      P_zz.transpose().ldlt().solve(P_xz.transpose()).transpose();
    
    State new_state = state + K*-res.mean;
    typename State::CovType new_cov = cov - K*res.cov*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
  
  template <int N, int NN>
  AugmentedState update(
      const boost::function<Matrix<double, N, 1> (StateWithMeasurementNoise<NN>)> &observe,
      const Matrix<double, NN, NN> &noise_cov) const {
    State const &state = static_cast<State const &>(*this);
    assert(noise_cov.rows() == noise_cov.cols());
    
    UnscentedTransform<Matrix<double, N, 1>, N,
      StateWithMeasurementNoise<NN>, StateWithMeasurementNoise<NN>::RowsAtCompileTime> res(
        observe,
        StateWithMeasurementNoise<NN>(
          state, Matrix<double, NN, 1>::Zero(noise_cov.rows())),
        StateWithMeasurementNoise<NN>::build_cov(cov, noise_cov));

    Matrix<double, State::RowsAtCompileTime, N> P_xz =
      res.cross_cov.transpose().template topLeftCorner(state.rows(), res.mean.rows());
    Matrix<double, N, N> P_zz = res.cov;
    
    //Matrix<double, State::RowsAtCompileTime, N> K = P_xz * P_zz.inverse();
    // instead, using matrix solver:
    // K = P_xz P_zz^-1
    // K P_zz = P_xz
    // P_zz' K' = P_xz'
    // K' = solve(P_zz', P_xz')
    // K = solve(P_zz', P_xz')'
    Matrix<double, State::RowsAtCompileTime, N> K =
      P_zz.transpose().ldlt().solve(P_xz.transpose()).transpose();
    
    State new_state = state + K*-res.mean;
    typename State::CovType new_cov = cov - K*res.cov*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
};


}

#endif
