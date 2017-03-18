#pragma once

#include <Eigen/Dense>
#include <tf/tf.h>

namespace uf_common {

template <class T>
inline T make_rgba(double r, double g, double b, double a) {
  T c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

template <class T>
inline T make_xyz(double x, double y, double z) {
  T p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

template <class T>
inline T vec2xyz(Eigen::Vector3d v) {
  return make_xyz<T>(v(0), v(1), v(2));
}
template <class T>
inline T vec2xyz(tf::Vector3 v) {
  return make_xyz<T>(v.x(), v.y(), v.z());
}
template <class T>
inline Eigen::Vector3d xyz2vec(T m) {
  return Eigen::Vector3d(m.x, m.y, m.z);
}

inline Eigen::Vector3d vec2vec(tf::Vector3 v) { return Eigen::Vector3d(v[0], v[1], v[2]); }
inline tf::Vector3 vec2vec(Eigen::Vector3d v) { return tf::Vector3(v[0], v[1], v[2]); }
inline Eigen::Quaterniond quat2quat(const tf::Quaternion &q) {
  return Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
}
inline tf::Quaternion quat2quat(const Eigen::Quaterniond &q) {
  return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

template <class T>
inline T make_xyzw(double x, double y, double z, double w) {
  T q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

template <class T>
inline T vec2xyzw(Eigen::Vector4d v) {
  return vec2xyzw<T>(v(0), v(1), v(2), v(3));
}
template <class T>
inline Eigen::Quaterniond xyzw2quat(T m) {
  return Eigen::Quaterniond(m.w, m.x, m.y, m.z);
}
template <class T>
inline T vec2wxyz(Eigen::Vector4d v) {
  return make_xyzw<T>(v(1), v(2), v(3), v(0));
}
inline tf::Quaternion vec2quat(Eigen::Vector4d v) { return tf::Quaternion(v[0], v[1], v[2], v[3]); }
template <class T>
inline T quat2xyzw(Eigen::Quaterniond q) {
  return make_xyzw<T>(q.x(), q.y(), q.z(), q.w());
}
}

