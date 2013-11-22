#ifndef GUARD_ZDOCIICJDATIGJFB
#define GUARD_ZDOCIICJDATIGJFB

#include <Eigen/Dense>
#include <boost/function.hpp>
#include <boost/optional.hpp>

#include <odom_estimator/util.h>
#include <odom_estimator/manifold.h>

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


template<typename OutPointType, typename InPointType>
GaussianDistributionWithCrossCov<OutPointType, InPointType>
unscented_transform(boost::function<OutPointType(InPointType)> const &func,
                    GaussianDistribution<InPointType> const &in,
                    double alpha = 1e-3,
                    double beta = 2,
                    double kappa = 0) {
  static const int OutVecLen = OutPointType::RowsAtCompileTime;
  static const int InVecLen = InPointType::RowsAtCompileTime;
  unsigned int in_vec_len = InVecLen != Dynamic ? InVecLen : in.cov.rows();
  
  unsigned int L = in_vec_len;
  double lambda = pow(alpha, 2)*(L + kappa) - L;
  
  SqMat<InVecLen> sqrt_cov =
    cholesky<InVecLen>((L + lambda)*in.cov);
  
  Vec<InVecLen> in_vecs[2*L+1];
  boost::optional<OutPointType> out_points[2*L+1];
  for(unsigned int i = 0; i <= 2*L; i++) {
    Vec<InVecLen> dx;
    if(i == 0) dx = Vec<InVecLen>::Zero(in_vec_len, 1);
    else if(i <= L) dx = sqrt_cov.col(i-1);
    else dx = -sqrt_cov.col(i-L-1);
    
    in_vecs[i] = dx;
    out_points[i] = func(in.mean + dx);
  }
  
  unsigned int out_vec_len = OutVecLen != Dynamic ? OutVecLen :
    (*out_points[0] - *out_points[0]).rows();
  Vec<OutVecLen> out_mean_minus_out_points_0 =
    Vec<OutVecLen>::Zero(out_vec_len, 1);
  for(unsigned int i = 0; i <= 2*L; i++) {
    double W_s = i == 0 ? lambda/(L + lambda) : 1./2/(L+lambda);
    out_mean_minus_out_points_0 += W_s * (*out_points[i] - *out_points[0]);
  }
  OutPointType out_mean = *out_points[0] + out_mean_minus_out_points_0;
  
  SqMat<OutVecLen> out_cov =
    SqMat<OutVecLen>::Zero(out_vec_len, out_vec_len);
  Mat<OutVecLen, InVecLen> out_cross_cov =
    Mat<OutVecLen, InVecLen>::Zero(out_vec_len, in_vec_len);
  for(unsigned int i = 0; i <= 2*L; i++) {
    double W_c = i == 0 ?
      lambda/(L + lambda) + (1 - pow(alpha, 2) + beta) : 1./2/(L+lambda);
    Vec<OutVecLen> dx = *out_points[i] - out_mean;
    out_cov += W_c * dx * dx.transpose();
    out_cross_cov += W_c * dx * in_vecs[i].transpose();
  }
  
  return GaussianDistributionWithCrossCov<OutPointType, InPointType>(
    GaussianDistribution<OutPointType>(out_mean, out_cov),
    out_cross_cov);
}


template<typename InType, typename OutType, typename ExtraType>
class IDistributionFunction {
  virtual GaussianDistribution<ExtraType> get_extra_distribution() const = 0;
  virtual OutType apply(InType const &input, ExtraType const &extra) const = 0;
public:
  virtual GaussianDistributionWithCrossCov<OutType, InType> operator()(GaussianDistribution<InType> const &input) const final {
    typedef ManifoldPair<InType, ExtraType> InAndExtraType;
    
    GaussianDistribution<ExtraType> extra = get_extra_distribution();
    
    GaussianDistributionWithCrossCov<OutType, InAndExtraType> res =
      unscented_transform<OutType, InAndExtraType>(
        [this](InAndExtraType const &x) {
          return apply(x.first, x.second);
        },
        GaussianDistribution<InAndExtraType>(
          InAndExtraType(input.mean, extra.mean),
          InAndExtraType::build_cov(input.cov, extra.cov)));
    
    return GaussianDistributionWithCrossCov<OutType, InType>(
      GaussianDistribution<OutType>(res.mean, res.cov),
      res.cross_cov.template topLeftCorner(res.mean.rows(), input.mean.rows()));
  }
};

template<typename InType, typename OutType, typename ExtraType>
class EasyDistributionFunction : public IDistributionFunction<InType, OutType, ExtraType> {
  std::function<OutType(InType, ExtraType)> func;
  GaussianDistribution<ExtraType> extra_distribution;
  GaussianDistribution<ExtraType> get_extra_distribution() const {
    return extra_distribution;
  }
  OutType apply(InType const &input, ExtraType const &extra) const {
    return func(input, extra);
  }
public:
  EasyDistributionFunction(std::function<OutType(InType, ExtraType)> func,
                           GaussianDistribution<ExtraType> const &extra_distribution) :
    func(func), extra_distribution(extra_distribution) {
  }
};



}

#endif
