#ifndef GUARD_KRBKTPTYOUPHLBSV
#define GUARD_KRBKTPTYOUPHLBSV

#include <odom_estimator/util.h>

namespace odom_estimator {

template<typename PointType>
struct GaussianDistribution {
  PointType mean;
  SqMat<PointType::RowsAtCompileTime> cov;
  
  GaussianDistribution(PointType const &mean,
                       SqMat<PointType::RowsAtCompileTime> const &cov) :
    mean(mean), cov(cov/2 + cov.transpose()/2) {
    assert(mean.rows() == cov.rows());
    assert(cov.cols() == cov.rows());
  }
};

template<typename PointType, typename VarType>
class GaussianDistributionWithCrossCov : public GaussianDistribution<PointType> {
  typedef Mat<PointType::RowsAtCompileTime, VarType::RowsAtCompileTime> CrossCovType;
public:
  CrossCovType cross_cov;
  
  GaussianDistributionWithCrossCov(GaussianDistribution<PointType> const &gd,
                                   CrossCovType const &cross_cov) :
    GaussianDistribution<PointType>(gd), cross_cov(cross_cov) {
  }
};

template<typename InType, typename OutType, typename ExtraType>
class IDistributionFunction {
  virtual GaussianDistribution<ExtraType> get_extra() const = 0;
  virtual OutType apply(InType const &input, ExtraType const &extra) const = 0;
public:
  virtual GaussianDistributionWithCrossCov<OutType, InType> operator()(GaussianDistribution<InType> const &input) const final {
    typedef ManifoldPair<InType, ExtraType> InAndExtraType;
    
    GaussianDistribution<ExtraType> extra = get_extra();
    
    UnscentedTransform<OutType, InAndExtraType> res(
        [this](InAndExtraType const &x) {
          return apply(x.first, x.second);
        },
        InAndExtraType(input.mean, extra.mean),
        InAndExtraType::build_cov(input.cov, extra.cov));
    
    return GaussianDistributionWithCrossCov<OutType, InType>(
      GaussianDistribution<OutType>(res.mean, res.cov),
      res.cross_cov.template topLeftCorner(res.mean.rows(), input.mean.rows()));
  }
};

template<typename DistributionFunction, typename InType>
GaussianDistribution<InType> kalman_thing(DistributionFunction const &df,
                                          GaussianDistribution<InType> const &input) {
  auto res = df(input);
  
  Matrix<double, InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> P_xz =
    res.cross_cov.transpose();
  Matrix<double, res.mean.RowsAtCompileTime, res.mean.RowsAtCompileTime> P_zz = res.cov;
  
  //Matrix<double, InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K = P_xz * P_zz.inverse();
  // instead, using matrix solver:
  // K = P_xz P_zz^-1
  // K P_zz = P_xz
  // P_zz' K' = P_xz'
  // K' = solve(P_zz', P_xz')
  // K = solve(P_zz', P_xz')'
  Matrix<double, InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K =
    P_zz.transpose().ldlt().solve(P_xz.transpose()).transpose();
  
  InType new_mean = input.mean + K*-res.mean;
  SqMat<InType::RowsAtCompileTime> new_cov = input.cov - K*res.cov*K.transpose();
  
  return GaussianDistribution<InType>(new_mean, new_cov);
}

template<typename InType, typename OutType, typename ExtraType>
class EasyDistributionFunction : public IDistributionFunction<InType, OutType, ExtraType> {
  std::function<OutType(InType, ExtraType)> func;
  GaussianDistribution<ExtraType> extra;
  GaussianDistribution<ExtraType> get_extra() const {
    return extra;
  }
  OutType apply(InType const &input, ExtraType const &extra) const {
    return func(input, extra);
  }
public:
  EasyDistributionFunction(std::function<OutType(InType, ExtraType)> func,
                           GaussianDistribution<ExtraType> const &extra) :
    func(func), extra(extra) {
  }
};

/*template<typename ErrorObserver> // ErrorObserver should implement IDistributionFunction
AugmentedState update(ErrorObserver const &ErrorObserver) const {
  typedef ManifoldPair<State, typename ErrorObserver::ExtraType> StateWithExtra;
  typedef typename ErrorObserver::ErrorType ErrorType;
  State const &state = static_cast<State const &>(*this);
  
  UnscentedTransform<ErrorType, ErrorType::RowsAtCompileTime,
    StateWithExtra, StateWithExtra::RowsAtCompileTime> res(
      [&ErrorObserver](StateWithExtra const &x) {
        return ErrorObserver.observe_error(x.first, x.second);
      },
      StateWithExtra(state, ErrorObserver.get_extra_mean()),
      StateWithExtra::build_cov(cov, ErrorObserver.get_extra_cov()));
  
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
}*/


}

#endif
