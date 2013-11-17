#ifndef _WHZHAUEWIGIZVWRS_
#define _WHZHAUEWIGIZVWRS_

#include <boost/math/special_functions/sinc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Vector3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace odom_estimator {


Eigen::Quaterniond quat_from_rotvec(Eigen::Vector3d r) {
    double angle = r.norm();
    Eigen::Quaterniond res;
    res.w() = cos(angle/2);
    res.vec() = boost::math::sinc_pi(angle/2)/2 * r; // = sin(angle/2) * r.normalized(), without the singularity
    return res;
}

Eigen::Vector3d rotvec_from_quat(Eigen::Quaterniond q) {
  q = q.normalized();
  if(q.w() < 0) q = Eigen::Quaterniond(-q.coeffs());
  return 2/boost::math::sinc_pi(acos(std::min(1., q.w()))) * q.vec();
}

Eigen::Quaterniond triad(Eigen::Vector3d v1_world, Eigen::Vector3d v2_world,
                  Eigen::Vector3d v1_body, Eigen::Vector3d v2_body) {
  Eigen::Matrix3d R_world_T;
  R_world_T.col(0) = v1_world.normalized();
  R_world_T.col(1) = v1_world.cross(v2_world).normalized();
  R_world_T.col(2) = v1_world.cross(R_world_T.col(1)).normalized();
  
  Eigen::Matrix3d R_body_T;
  R_body_T.col(0) = v1_body.normalized();
  R_body_T.col(1) = v1_body.cross(v2_body).normalized();
  R_body_T.col(2) = v1_body.cross(R_body_T.col(1)).normalized();

  // R_body_T^-1 = R_T_body, but since they are orthogonal
  // R_body_T^-1 = R_body_T'
  Eigen::Matrix3d R_world_body = R_world_T * R_body_T.transpose();

  return Eigen::Quaterniond(R_world_body);
}

Eigen::Matrix<double, 1, 1> scalar_matrix(double x) {
  return (Eigen::Matrix<double, 1, 1>() << x).finished();
}

template<int N>
Eigen::Matrix<double, N, N> cholesky(Eigen::Matrix<double, N, N> x) {
  Eigen::LDLT<Eigen::Matrix<double, N, N> > ldlt = x.ldlt();
  return ldlt.transpositionsP().transpose() * Eigen::Matrix<double, N, N>(ldlt.matrixL()) * Eigen::Matrix<double, N, 1>(ldlt.vectorD().array().sqrt()).asDiagonal();
}

inline Eigen::Vector3d xyz2vec(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d res; tf::vectorMsgToEigen(msg, res);
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
