#ifndef GUARD_KRBKTPTYOUPHLBSV
#define GUARD_KRBKTPTYOUPHLBSV

#include <odom_estimator/util.h>
#include <odom_estimator/unscented_transform.h>

namespace odom_estimator {



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



}

#endif
