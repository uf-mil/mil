#include <boost/math/special_functions/sinc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

template <class T>
inline T make_xyz(double x, double y, double z) {
    T p;
    p.x = x; p.y = y; p.z = z;
    return p;
}
template <class T>
inline T vec2xyz(Eigen::Vector3d v) {
    return make_xyz<T>(v(0), v(1), v(2));
}
template <class T>
inline Eigen::Vector3d xyz2vec(T m) {
    return Eigen::Vector3d(m.x, m.y, m.z);
}
template <class T>
inline T make_xyzw(double x, double y, double z, double w) {
    T q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}
template <class T>
inline T quat2xyzw(Eigen::Quaterniond q) {
    return make_xyzw<T>(q.x(), q.y(), q.z(), q.w());
}


Eigen::Quaterniond quat_from_rotvec(Eigen::Vector3d r) {
    double angle = r.norm();
    Eigen::Quaterniond res;
    res.w() = cos(angle/2);
    res.vec() = boost::math::sinc_pi(angle/2)/2 * r; // = sin(angle/2) * r.normalized(), without the singularity
    return res;
}

Eigen::Vector3d rotvec_from_quat(Eigen::Quaterniond q) {
  q = q.normalized();
  if(q.w() < 0) q = Eigen::Quaterniond(-q.x(), -q.y(), -q.z(), -q.w());
  return 2/boost::math::sinc_pi(acos(q.w())) * q.vec();
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
