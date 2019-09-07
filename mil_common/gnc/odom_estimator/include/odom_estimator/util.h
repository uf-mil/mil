#ifndef GUARD_MJABIWMKOFIHVAOG
#define GUARD_MJABIWMKOFIHVAOG

#include <boost/math/special_functions/sinc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <tf_conversions/tf_eigen.h>

namespace odom_estimator
{
template <int N, typename T = double>
using Vec = Eigen::Matrix<T, N, 1>;

template <int M, int N, typename T = double>
using Mat = Eigen::Matrix<T, M, N>;

template <int N, typename T = double>
using SqMat = Eigen::Matrix<T, N, N>;

using Eigen::Dynamic;

typedef Eigen::Quaterniond Quaternion;

static inline constexpr int addRowsAtCompileTime(int a, int b)
{
  return a == Dynamic ? Dynamic : b == Dynamic ? Dynamic : a + b;
}

Quaternion quat_from_rotvec(Vec<3> r)
{
  double angle = r.norm();
  Quaternion res;
  res.w() = cos(angle / 2);
  res.vec() = boost::math::sinc_pi(angle / 2) / 2 * r;  // = sin(angle/2) * r.normalized(), without the singularity
  return res;
}

Vec<3> rotvec_from_quat(Quaternion q)
{
  q = q.normalized();
  if (q.w() < 0)
    q = Quaternion(-q.coeffs());
  return 2 / boost::math::sinc_pi(acos(std::min(1., q.w()))) * q.vec();
}

Quaternion triad(Vec<3> v1_world, Vec<3> v2_world, Vec<3> v1_body, Vec<3> v2_body)
{
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

SqMat<1> scalar_matrix(double x)
{
  return (SqMat<1>() << x).finished();
}

template <typename Derived1, typename Derived2>
SqMat<addRowsAtCompileTime(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime)>
joinDiagonally(Eigen::EigenBase<Derived1> const &a, Eigen::EigenBase<Derived2> const &b)
{
  static_assert(Derived1::RowsAtCompileTime == Derived1::ColsAtCompileTime, "a must be square");
  static_assert(Derived2::RowsAtCompileTime == Derived2::ColsAtCompileTime, "a must be square");
  assert(a.rows() == a.cols() && b.rows() == b.cols());
  SqMat<addRowsAtCompileTime(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime)> res =
      SqMat<addRowsAtCompileTime(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime)>::Zero(a.rows() + b.rows(),
                                                                                                  a.cols() + b.cols());
  res.topLeftCorner(a.rows(), a.cols()) = a.derived();
  res.bottomRightCorner(b.rows(), b.cols()) = b.derived();
  return res;
}

template <int N>
SqMat<N> cholesky(SqMat<N> x)
{
  Eigen::LDLT<SqMat<N> > ldlt = x.ldlt();
  return ldlt.transpositionsP().transpose() * SqMat<N>(ldlt.matrixL()) *
         Vec<N>(ldlt.vectorD().array().sqrt()).asDiagonal();
}

inline Vec<3> xyz2vec(const geometry_msgs::Vector3 &msg)
{
  Vec<3> res;
  tf::vectorMsgToEigen(msg, res);
  return res;
}
inline Vec<3> point2vec(const geometry_msgs::Point &msg)
{
  Vec<3> res;
  tf::pointMsgToEigen(msg, res);
  return res;
}

template <typename Derived>
void assert_none_nan(const Eigen::MatrixBase<Derived> &m)
{
  for (unsigned int i = 0; i < m.rows(); i++)
  {
    for (unsigned int j = 0; j < m.cols(); j++)
    {
      assert(std::isfinite(m(i, j)));
    }
  }
}
}

#endif
