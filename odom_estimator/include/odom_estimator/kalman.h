#ifndef GUARD_KRBKTPTYOUPHLBSV
#define GUARD_KRBKTPTYOUPHLBSV

#include <odom_estimator/util.h>

namespace odom_estimator {


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
  
  Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> P_xz =
    res.cross_cov.transpose();
  Mat<res.mean.RowsAtCompileTime, res.mean.RowsAtCompileTime> P_zz = res.cov;
  
  //Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K = P_xz * P_zz.inverse();
  // instead, using matrix solver:
  // K = P_xz P_zz^-1
  // K P_zz = P_xz
  // P_zz' K' = P_xz'
  // K' = solve(P_zz', P_xz')
  // K = solve(P_zz', P_xz')'
  Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K =
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



}

#endif
