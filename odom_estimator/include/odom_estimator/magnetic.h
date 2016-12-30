#ifndef GUARD_HVRLSXDXNEOHTCRV
#define GUARD_HVRLSXDXNEOHTCRV

#include <sstream>
#include <fstream>

#include <boost/foreach.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include <ros/package.h>

#include "odom_estimator/util.h"
#include "odom_estimator/earth.h"


namespace odom_estimator {
namespace magnetic {


// computes pairs
//   res[n] = (cos(n theta), sin(n theta))
// with n from 0 to max_n, given x = cos(theta) and y = sin(theta).
template<typename T>
std::vector<std::pair<T, T> > compute_cos_sin(T x, T y, int max_n) {
  std::vector<std::pair<T, T> > res;
  res.push_back(std::make_pair(T(1), T(0)));
  res.push_back(std::make_pair(x, y));
  for(int n = 2; n <= max_n; n++) {
    res.push_back(std::make_pair(
      T(2) * x * res[n-1].first  - res[n-2].first,
      T(2) * x * res[n-1].second - res[n-2].second));
  }
  return res;
}


// computes a table of values:
//   result(m, k) = \tilde{P}_k^{(m,m)}(x)
// of the Jacobi polynomials P_k^{(m,m)}(x) normalized (divided) by
//   \sqrt{\dfrac{2^{2m+1}\Gamma^2(k+m+1)}
//               {(2k+2m+1)\Gamma(k+1)\Gamma(k+2m+1)}}
// using a stable recurrence relation.
// Source: http://www.ohio.edu/people/mohlenka/research/uguide.pdf section 3.1
//   (mirrored at doc/uguide.pdf)
template <typename T>
Eigen::Matrix<T, Dynamic, Dynamic>
normalized_jacobi(int max_m, int max_k, T x) {
  Eigen::Matrix<T, Dynamic, Dynamic> result(max_m+1, max_k+1);
  for(int m = 0; m <= max_m; m++) {
    if(m == 0) {
      result(m, 0) = T(1/sqrt(2));
    } else {
      result(m, 0) = result(m-1, 0)*sqrt(1+1/(2.*m));
    }
    for(int k = 1; k <= max_k; k++) {
      T preprek = k == 1 ? T(0) : result(m, k-2);
      T prek = result(m, k-1);
      result(m, k) = T(2)*x*prek*sqrt((1 + (m-1./2)/k)*(1-(m-1./2)/(k+2*m))) -
        preprek*sqrt((1+4/(2.*k+2.*m-3))*(1-1./k)*(1-1/(k+2.*m)));
    }
  }
  return result;
}
// computes a table of values:
//   result(m, n) = \breve{P}_n^m(arg)
// of the Schmidt semi-normalized associated Legendre functions.
// See http://www.ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010_Report.pdf
//   section 1.2
template<typename T>
Eigen::Matrix<T, Dynamic, Dynamic>
semi_normalized_associated_legendre(int max_m, int max_n, T arg) {
  Eigen::Matrix<T, Dynamic, Dynamic> p =
    normalized_jacobi<T>(max_m, max_n, arg);
  Eigen::Matrix<T, Dynamic, Dynamic> result(max_m+1, max_n+1);
  T sqrt1marg2 = sqrt(T(1)-arg*arg);
  T pow_sqrt1marg2_m = T(1);
  for(int m = 0; m <= max_m; m++) {
    for(int n = 0; n <= max_n; n++) {
      if(n-m < 0) {
        result(m, n) = T(NAN);
        continue;
      }
      result(m, n) = pow_sqrt1marg2_m * p(m, n-m)/sqrt((2*n+1)/2.);
      if(m > 0) {
        result(m, n) *= sqrt(2);
      }
    }
    pow_sqrt1marg2_m *= sqrt1marg2;
  }
  return result;
}


struct Coeff {
  int n, m;
  double g, h, gdot, hdot;
  
  Coeff(std::string const &line) {
    std::istringstream ss(line);
    ss.exceptions(std::ios::eofbit | std::ios::failbit | std::ios::badbit);
    ss >> n >> m >> g >> h >> gdot >> hdot;
  }
};

class MagneticModel {
  std::vector<Coeff> coeffs;
  int max_n, max_m;
  double t0_year;

public:
  MagneticModel(std::string const &filename) :
    max_n(0), max_m(0) {
    std::ifstream f(filename);
    
    std::string header; getline(f, header);
    std::istringstream header_ss(header);
    header_ss.exceptions(
      std::ios::eofbit | std::ios::failbit | std::ios::badbit);
    header_ss >> t0_year;
    
    while(true) {
      std::string line; getline(f, line);
      if(line.substr(0, 10) == "9999999999") break;
      
      Coeff const coeff(line);
      coeffs.push_back(coeff);
      max_n = std::max(max_n, coeff.n);
      max_m = std::max(max_m, coeff.m);
    }
  }

  template<typename T>
  T getPotential(Vec<3, T> pos_eci, double t) const {
    // would use ecef_from_inertial, but it's not templated to work here
    Vec<3, T> pos_ecef = quat_from_rotvec(-w_E * t).toRotationMatrix().cast<T>() * pos_eci;
    double t_year = (t - 1262322000)/(365.25*86400) + 2010;
    
    T a = T(6371200);
    T r = pos_ecef.norm();
    std::vector<std::pair<T, T> > cos_sin = compute_cos_sin<T>(
      pos_ecef(0) / Vec<2, T>(pos_ecef(0), pos_ecef(1)).norm(),
      pos_ecef(1) / Vec<2, T>(pos_ecef(0), pos_ecef(1)).norm(),
      max_m);
    Eigen::Matrix<T, Dynamic, Dynamic> p =
      semi_normalized_associated_legendre<T>(max_m, max_n, pos_ecef(2)/r);
    
    T pow_aoverr_n[max_n+2];
    pow_aoverr_n[0] = T(1);
    for(unsigned int i = 1; i < max_n+2; i++) {
      pow_aoverr_n[i] = pow_aoverr_n[i-1] * T(a/r);
    }
    
    T res = T(0);
    BOOST_FOREACH(Coeff const &coeff, coeffs) {
      res += a * (
        T(coeff.g + coeff.gdot * (t_year - t0_year)) * cos_sin[coeff.m].first +
        T(coeff.h + coeff.hdot * (t_year - t0_year)) * cos_sin[coeff.m].second
      ) * pow_aoverr_n[coeff.n+1] * p(coeff.m, coeff.n);
    }
    return res*1e-9; // convert nT to T
  }
  
  Vec<3> getField(Vec<3> pos_eci, double t) const {
    typedef Eigen::AutoDiffScalar<Vec<3> > ScalarWithGradient;
    ScalarWithGradient x = getPotential<ScalarWithGradient>(
      Vec<3, ScalarWithGradient>(
        ScalarWithGradient(pos_eci(0), Vec<3>(1, 0, 0)),
        ScalarWithGradient(pos_eci(1), Vec<3>(0, 1, 0)),
        ScalarWithGradient(pos_eci(2), Vec<3>(0, 0, 1))),
      t);
    return -x.derivatives();
  }
  
  std::pair<Vec<3>, SqMat<3> > getFieldAndJacobian(Vec<3> pos_eci, double t) const {
    typedef Eigen::AutoDiffScalar<Vec<3> > ScalarWithGradient;
    typedef Vec<3, ScalarWithGradient> VectorWithJacobian;
    typedef Eigen::AutoDiffScalar<VectorWithJacobian> ScalarWithHessian;
    typedef Vec<3, ScalarWithHessian> VectorWithHessian;
    struct make {
      static ScalarWithHessian func(double val, int i) {
        return ScalarWithHessian(ScalarWithGradient(val, 3, i), (VectorWithJacobian() <<
          ScalarWithGradient(i == 0),
          ScalarWithGradient(i == 1),
          ScalarWithGradient(i == 2)).finished());
      }
    };
    ScalarWithHessian x = getPotential<ScalarWithHessian>(
      VectorWithHessian(
        make::func(pos_eci(0), 0),
        make::func(pos_eci(1), 1),
        make::func(pos_eci(2), 2)),
      t);
    Vec<3> y;
    SqMat<3> res;
    for(int i = 0; i < 3; i++) {
      y(i) = -x.derivatives()(i).value();
      res.row(i) = x.derivatives()(i).derivatives();
    }
    return std::make_pair(y, res);
  }
};


}
}

#endif
