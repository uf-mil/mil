#ifndef GUARD_ZDOCIICJDATIGJFB
#define GUARD_ZDOCIICJDATIGJFB

#include <Eigen/Dense>
#include <boost/function.hpp>
#include <boost/optional.hpp>

#include "util.h"
#include "math.h"

namespace odom_estimator {


using namespace Eigen;

template<class OutPointType, int OutVecLen,
         class  InPointType, int  InVecLen>
struct UnscentedTransform {
  OutPointType mean;
  SqMat<OutVecLen> cov;
  Mat<OutVecLen, InVecLen> cross_cov;
  
  UnscentedTransform(const boost::function<OutPointType(InPointType)> &func,
                     const InPointType &mean,
                     const SqMat<InVecLen> &cov,
                     double alpha = 1e-3,
                     double beta = 2,
                     double kappa = 0) :
    mean(func(mean)) {
    unsigned int in_vec_len = InVecLen != Dynamic ? InVecLen : cov.rows();
    
    unsigned int L = in_vec_len;
    double lambda = pow(alpha, 2)*(L + kappa) - L;
    
    SqMat<InVecLen> sqrt_cov =
      cholesky<InVecLen>((L + lambda)*cov);
    
    Vec<InVecLen> in_vecs[2*L+1];
    boost::optional<OutPointType> out_points[2*L+1];
    for(unsigned int i = 0; i <= 2*L; i++) {
      Vec<InVecLen> dx;
      if(i == 0) dx = Vec<InVecLen>::Zero(in_vec_len, 1);
      else if(i <= L) dx = sqrt_cov.col(i-1);
      else dx = -sqrt_cov.col(i-L-1);
      
      in_vecs[i] = dx;
      out_points[i] = func(mean + dx);
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
    
    this->mean = out_mean;
    this->cov = out_cov;
    this->cross_cov = out_cross_cov;
  }
};

template<typename First, typename Second>
class ManifoldPair {
public:
  static int const RowsAtCompileTime = 
    First::RowsAtCompileTime == Dynamic ? Dynamic :
    Second::RowsAtCompileTime == Dynamic ? Dynamic :
    First::RowsAtCompileTime + Second::RowsAtCompileTime;
private:
  typedef Vec<RowsAtCompileTime> DeltaType;
  typedef SqMat<RowsAtCompileTime> CovType;
public:
  First const first;
  Second const second;
  ManifoldPair(First const &first, Second const &second) :
    first(first), second(second) {
  }
  
  unsigned int rows() const {
    return first.rows() + second.rows();
  }
  DeltaType operator-(const ManifoldPair<First, Second> &other) const {
    return (DeltaType() <<
      first - other.first,
      second - other.second).finished();
  }
  ManifoldPair<First, Second> operator+(const DeltaType &other) const {
    return ManifoldPair<First, Second>(
      first + other.head(first.rows()),
      second + other.tail(second.rows()));
  }
  
  static CovType build_cov(
      SqMat<First::RowsAtCompileTime> const &first_cov,
      SqMat<Second::RowsAtCompileTime> const &second_cov) {
    CovType res = CovType::Zero(first_cov.rows() + second_cov.rows(),
                                first_cov.rows() + second_cov.rows());
    res.topLeftCorner(first_cov.rows(), first_cov.rows()) = first_cov;
    res.bottomRightCorner(second_cov.rows(), second_cov.rows()) = second_cov;
    return res;
  }
};

}

#endif
