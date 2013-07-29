#include <Eigen/Dense>
#include <boost/function.hpp>
#include <boost/optional.hpp>

#include "util.h"

using namespace Eigen;

template<class OutPointType, int OutVecLen,
         class  InPointType, int  InVecLen>
struct UnscentedTransform {
  OutPointType mean;
  Matrix<double, OutVecLen, OutVecLen> cov;
  Matrix<double, OutVecLen, InVecLen> cross_cov;
  
  UnscentedTransform(const boost::function<OutPointType(InPointType)> &func,
                     const InPointType &mean,
                     const Matrix<double, InVecLen, InVecLen> &cov) :
    mean(func(mean)) {
    unsigned int in_vec_len = InVecLen != Dynamic ? InVecLen : cov.rows();
    
    unsigned int L = in_vec_len;
    double alpha = 1e-3;
    double beta = 2;
    double kappa = 0;
    double lambda = pow(alpha, 2)*(L + kappa) - L;
    
    Matrix<double, InVecLen, InVecLen> sqrt_cov =
      cholesky<InVecLen>((L + lambda)*cov);
    
    Matrix<double, InVecLen, 1> in_vecs[2*L+1];
    boost::optional<OutPointType> out_points[2*L+1];
    for(unsigned int i = 0; i <= 2*L; i++) {
      Matrix<double, InVecLen, 1> dx;
      if(i == 0) dx = Matrix<double, InVecLen, 1>::Zero(in_vec_len, 1);
      else if(i <= L) dx = sqrt_cov.col(i-1);
      else dx = -sqrt_cov.col(i-L-1);
      
      in_vecs[i] = dx;
      out_points[i] = func(mean + dx);
    }
    
    unsigned int out_vec_len = OutVecLen != Dynamic ? OutVecLen :
      (*out_points[0] - *out_points[0]).rows();
    Matrix<double, OutVecLen, 1> out_mean_minus_out_points_0 =
      Matrix<double, OutVecLen, 1>::Zero(out_vec_len, 1);
    for(unsigned int i = 0; i <= 2*L; i++) {
      double W_s = i == 0 ? lambda/(L + lambda) : 1./2/(L+lambda);
      out_mean_minus_out_points_0 += W_s * (*out_points[i] - *out_points[0]);
    }
    OutPointType out_mean = *out_points[0] + out_mean_minus_out_points_0;
    
    Matrix<double, OutVecLen, OutVecLen> out_cov =
      Matrix<double, OutVecLen, OutVecLen>::Zero(out_vec_len, out_vec_len);
    Matrix<double, OutVecLen, InVecLen> out_cross_cov =
      Matrix<double, OutVecLen, InVecLen>::Zero(out_vec_len, in_vec_len);
    for(unsigned int i = 0; i <= 2*L; i++) {
      double W_c = i == 0 ?
        lambda/(L + lambda) + (1 - pow(alpha, 2) + beta) : 1./2/(L+lambda);
      Matrix<double, OutVecLen, 1> dx = *out_points[i] - out_mean;
      out_cov += W_c * dx * dx.transpose();
      out_cross_cov += W_c * dx * in_vecs[i].transpose();
    }
    
    this->mean = out_mean;
    this->cov = out_cov;
    this->cross_cov = out_cross_cov;
  }
};
