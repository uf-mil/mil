#ifndef GUARD_KRBKTPTYOUPHLBSV
#define GUARD_KRBKTPTYOUPHLBSV

#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"

namespace odom_estimator
{
// does a kalman update given error function `df`, which should return the
// distribution of the difference between predicted values and measured values
template <typename InType, typename ErrorType>
GaussianDistribution<InType> kalman_update(IDistributionFunction<InType, ErrorType> const &df,
                                           GaussianDistribution<InType> const &input)
{
  GaussianDistributionWithCrossCov<ErrorType, InType> res = df(input);

  Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> P_xz = res.cross_cov.transpose();
  Mat<res.mean.RowsAtCompileTime, res.mean.RowsAtCompileTime> P_zz = res.cov;

  // Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K =
  //  P_xz * P_zz.inverse();
  // instead, using matrix solver:
  // K = P_xz P_zz^-1
  // K P_zz = P_xz
  // P_zz' K' = P_xz'
  // K' = solve(P_zz', P_xz')
  // K = solve(P_zz', P_xz')'
  if (P_zz.rows() == 0)
  {
    return input;  // Eigen::MatrixBase::ldlt() crashes for zero-sized matrices
  }
  Mat<InType::RowsAtCompileTime, res.mean.RowsAtCompileTime> K =
      P_zz.transpose().ldlt().solve(P_xz.transpose()).transpose();

  InType new_mean = input.mean + K * -res.mean;
  SqMat<InType::RowsAtCompileTime> new_cov = input.cov - K * res.cov * K.transpose();

  return GaussianDistribution<InType>(new_mean, new_cov);
}
}

#endif
