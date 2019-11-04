#ifndef GUARD_ZDOCIICJDATIGJFB
#define GUARD_ZDOCIICJDATIGJFB

#include <Eigen/Dense>
#include <boost/function.hpp>
#include <boost/optional.hpp>

#include "odom_estimator/manifold.h"
#include "odom_estimator/util.h"

namespace odom_estimator
{
// is a gaussian distribution on an arbitrary manifold
template <typename PointType>
struct GaussianDistribution
{
  PointType mean;
  SqMat<PointType::RowsAtCompileTime> cov;

  GaussianDistribution(PointType const &mean, SqMat<PointType::RowsAtCompileTime> const &cov)
    : mean(mean), cov(cov / 2 + cov.transpose() / 2)
  {
    assert(cov.rows() == mean.rows());
    assert(cov.cols() == mean.rows());
  }
};

// is a GaussianDistribution that also has cross-covariance information
// between this and another implied distribution
template <typename PointType, typename CrossPointType>
class GaussianDistributionWithCrossCov : public GaussianDistribution<PointType>
{
  typedef Mat<PointType::RowsAtCompileTime, CrossPointType::RowsAtCompileTime> CrossCovType;

public:
  CrossCovType cross_cov;  // = E[(this - this.mean) * (other - other.mean)^T]

  GaussianDistributionWithCrossCov(GaussianDistribution<PointType> const &gd, CrossCovType const &cross_cov)
    : GaussianDistribution<PointType>(gd), cross_cov(cross_cov)
  {
    assert(cross_cov.rows() == this->mean.rows());
  }
};

// approximately transforms a distribution, `in`, by a function, `func`,
// resulting in a new distribution (along with the cross-correlation with the
// original distribution)
template <typename OutPointType, typename InPointType>
GaussianDistributionWithCrossCov<OutPointType, InPointType>
unscented_transform(boost::function<OutPointType(InPointType)> const &func, GaussianDistribution<InPointType> const &in,
                    double alpha = 1e-3, double beta = 2, double kappa = 0)
{
  static const int OutVecLen = OutPointType::RowsAtCompileTime;
  static const int InVecLen = InPointType::RowsAtCompileTime;
  unsigned int in_vec_len = InVecLen != Dynamic ? InVecLen : in.cov.rows();

  unsigned int L = in_vec_len;
  double lambda = pow(alpha, 2) * (L + kappa) - L;

  SqMat<InVecLen> sqrt_cov = cholesky<InVecLen>((L + lambda) * in.cov);

  Vec<InVecLen> in_vecs[2 * L + 1];
  boost::optional<OutPointType> out_points[2 * L + 1];
  for (unsigned int i = 0; i <= 2 * L; i++)
  {
    Vec<InVecLen> dx;
    if (i == 0)
      dx = Vec<InVecLen>::Zero(in_vec_len, 1);
    else if (i <= L)
      dx = sqrt_cov.col(i - 1);
    else
      dx = -sqrt_cov.col(i - L - 1);

    in_vecs[i] = dx;
    out_points[i] = func(in.mean + dx);
  }

  unsigned int out_vec_len = OutVecLen != Dynamic ? OutVecLen : (*out_points[0] - *out_points[0]).rows();
  Vec<OutVecLen> out_mean_minus_out_points_0 = Vec<OutVecLen>::Zero(out_vec_len, 1);
  for (unsigned int i = 0; i <= 2 * L; i++)
  {
    double W_s = i == 0 ? lambda / (L + lambda) : 1. / 2 / (L + lambda);
    out_mean_minus_out_points_0.noalias() += W_s * (*out_points[i] - *out_points[0]);
  }
  OutPointType out_mean = *out_points[0] + out_mean_minus_out_points_0;

  SqMat<OutVecLen> out_cov = SqMat<OutVecLen>::Zero(out_vec_len, out_vec_len);
  Mat<OutVecLen, InVecLen> out_cross_cov = Mat<OutVecLen, InVecLen>::Zero(out_vec_len, in_vec_len);
  for (unsigned int i = 0; i <= 2 * L; i++)
  {
    double W_c = i == 0 ? lambda / (L + lambda) + (1 - pow(alpha, 2) + beta) : 1. / 2 / (L + lambda);
    Vec<OutVecLen> dx = *out_points[i] - out_mean;
    out_cov.noalias() += W_c * dx * dx.transpose();
    out_cross_cov.noalias() += W_c * dx * in_vecs[i].transpose();
  }

  return GaussianDistributionWithCrossCov<OutPointType, InPointType>(
      GaussianDistribution<OutPointType>(out_mean, out_cov), out_cross_cov);
}

// is an interface describing functors that take a GaussianDistribution and
// produce a GaussianDistributionWithCrossCov, possibly with distinct point
// types
template <typename InType, typename OutType>
class IDistributionFunction
{
public:
  virtual GaussianDistributionWithCrossCov<OutType, InType>
  operator()(GaussianDistribution<InType> const &input) const = 0;
};

// is an implementation of IDistributionFunction that allows subclasses to
// fill in a point propagation function and then uses it to transform
// distributions using the unscented transform
template <typename InType, typename OutType, typename ExtraType>
class UnscentedTransformDistributionFunction : public IDistributionFunction<InType, OutType>
{
public:
  virtual GaussianDistribution<ExtraType> get_extra_distribution() const = 0;
  virtual OutType apply(InType const &input, ExtraType const &extra) const = 0;

  GaussianDistributionWithCrossCov<OutType, InType>
  operator()(GaussianDistribution<InType> const &input) const final override
  {
    typedef ManifoldPair<InType, ExtraType> InAndExtraType;

    GaussianDistribution<ExtraType> extra = get_extra_distribution();

    GaussianDistributionWithCrossCov<OutType, InAndExtraType> res = unscented_transform<OutType, InAndExtraType>(
        [this](InAndExtraType const &x) { return apply(x.first, x.second); },
        GaussianDistribution<InAndExtraType>(InAndExtraType(input.mean, extra.mean),
                                             joinDiagonally(input.cov, extra.cov)));

    return GaussianDistributionWithCrossCov<OutType, InType>(
        GaussianDistribution<OutType>(res.mean, res.cov),
        res.cross_cov.template topLeftCorner(res.mean.rows(), input.mean.rows()));
  }
};

// is a specialization of UnscentedTransformDistributionFunction that allows
// the point propagation function to be provided as an argument
template <typename InType, typename OutType, typename ExtraType = Vec<0>>
class EasyDistributionFunction : public UnscentedTransformDistributionFunction<InType, OutType, ExtraType>
{
  std::function<OutType(InType, ExtraType)> func;
  typedef GaussianDistribution<ExtraType> ExtraDistributionType;
  ExtraDistributionType extra_distribution;

public:
  ExtraDistributionType get_extra_distribution() const override
  {
    return extra_distribution;
  }
  OutType apply(InType const &input, ExtraType const &extra) const override
  {
    return func(input, extra);
  }
  EasyDistributionFunction(std::function<OutType(InType, ExtraType)> func,
                           ExtraDistributionType const &extra_distribution = ExtraDistributionType(ExtraType(),
                                                                                                   SqMat<0>()))
    : func(func), extra_distribution(extra_distribution)
  {
  }
};
}

#endif
