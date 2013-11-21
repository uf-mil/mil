#ifndef GUARD_MJABIWMKOFIHVAOG
#define GUARD_MJABIWMKOFIHVAOG

#include <boost/math/special_functions/sinc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Vector3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace odom_estimator {


template<int N>
using Vec = Eigen::Matrix<double, N, 1>;

template<int M, int N>
using Mat = Eigen::Matrix<double, M, N>;

template<int N>
using SqMat = Eigen::Matrix<double, N, N>;

using Eigen::Dynamic;

typedef Eigen::Quaterniond Quaternion;


Quaternion quat_from_rotvec(Vec<3> r) {
    double angle = r.norm();
    Quaternion res;
    res.w() = cos(angle/2);
    res.vec() = boost::math::sinc_pi(angle/2)/2 * r; // = sin(angle/2) * r.normalized(), without the singularity
    return res;
}

Vec<3> rotvec_from_quat(Quaternion q) {
  q = q.normalized();
  if(q.w() < 0) q = Quaternion(-q.coeffs());
  return 2/boost::math::sinc_pi(acos(std::min(1., q.w()))) * q.vec();
}

Quaternion triad(Vec<3> v1_world, Vec<3> v2_world,
                 Vec<3> v1_body, Vec<3> v2_body) {
  SqMat<3> R_world_T;
  R_world_T.col(0) = v1_world.normalized();
  R_world_T.col(1) = v1_world.cross(v2_world).normalized();
  R_world_T.col(2) = v1_world.cross(R_world_T.col(1)).normalized();
  
  SqMat<3> R_body_T;
  R_body_T.col(0) = v1_body.normalized();
  R_body_T.col(1) = v1_body.cross(v2_body).normalized();
  R_body_T.col(2) = v1_body.cross(R_body_T.col(1)).normalized();

  // R_body_T^-1 = R_T_body, but since they are orthogonal
  // R_body_T^-1 = R_body_T'
  SqMat<3> R_world_body = R_world_T * R_body_T.transpose();

  return Quaternion(R_world_body);
}

SqMat<1> scalar_matrix(double x) {
  return (SqMat<1>() << x).finished();
}

template<int N>
SqMat<N> cholesky(SqMat<N> x) {
  Eigen::LDLT<SqMat<N> > ldlt = x.ldlt();
  return ldlt.transpositionsP().transpose() * SqMat<N>(ldlt.matrixL()) * Vec<N>(ldlt.vectorD().array().sqrt()).asDiagonal();
}

inline Vec<3> xyz2vec(const geometry_msgs::Vector3 &msg) {
  Vec<3> res; tf::vectorMsgToEigen(msg, res);
  return res;
}
inline Vec<3> point2vec(const geometry_msgs::Point &msg) {
  Vec<3> res; tf::pointMsgToEigen(msg, res);
  return res;
}

template<typename Derived>
void assert_none_nan(const Eigen::MatrixBase<Derived> &m) {
  for(unsigned int i = 0; i < m.rows(); i++) {
    for(unsigned int j = 0; j < m.cols(); j++) {
      assert(std::isfinite(m(i, j)));
    }
  }
}


}

#endif
