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
  
  template <int N, int NN>
  AugmentedState update(
      const boost::function<Matrix<double, N, 1> (StateWithMeasurementNoise<NN>)> &observe,
      const Matrix<double, NN, NN> &noise_cov) const {
    unsigned int realNN = NN != Dynamic ? NN : noise_cov.rows();
    assert(noise_cov.rows() == noise_cov.cols());
    
    StateWithMeasurementNoise<NN> mean = StateWithMeasurementNoise<NN>(
      static_cast<const State&>(*this), Matrix<double, NN, 1>::Zero(realNN));
    typedef Matrix<double, NN != Dynamic ? State::RowsAtCompileTime + NN : Dynamic,
                           NN != Dynamic ? State::RowsAtCompileTime + NN : Dynamic>
      AugmentedMatrixType;
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero(
      State::RowsAtCompileTime + realNN, State::RowsAtCompileTime + realNN);
    Pa.template block<State::RowsAtCompileTime, State::RowsAtCompileTime>(0, 0) = cov;
    Pa.template bottomRightCorner(realNN, realNN) = noise_cov;
    
    UnscentedTransform<Matrix<double, N, 1>, N,
      StateWithMeasurementNoise<NN>, NN != Dynamic ? State::RowsAtCompileTime + NN : Dynamic>
      res(observe, mean, Pa);
    unsigned int realN = res.mean.rows();
    
    //Matrix<double, State::RowsAtCompileTime, N> K =
    //  res.cross_cov.transpose().template topLeftCorner(State::RowsAtCompileTime, realN) *
    //  res.cov.inverse();

    // a = res.cross_cov.transpose().template topLeftCorner(State::RowsAtCompileTime, realN)
    // b = res.cov
    // K = a b^-1
    // K b = a
    // b' K' = a'
    // K' = solve(b', a')
    // K = solve(b', a')'
    Matrix<double, State::RowsAtCompileTime, N> K =
      res.cov.transpose().ldlt().solve(
        res.cross_cov.transpose()
          .template topLeftCorner(State::RowsAtCompileTime, realN).transpose()
      ).transpose();
    
    State new_state = static_cast<const State&>(*this) + K*-res.mean;
    typename State::CovType new_cov = cov - K*res.cov*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
};


}

#endif
