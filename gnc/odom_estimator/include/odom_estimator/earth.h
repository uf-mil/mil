#ifndef GUARD_AITWOYBVCHUJJFBI
#define GUARD_AITWOYBVCHUJJFBI

#include <Eigen/Geometry>

#include "odom_estimator/gravity.h"

namespace odom_estimator {


static const Vec<3> w_E(0, 0, 0.729211510e-4); // from glonass

Vec<3> inertial_from_ecef(double t, Vec<3> ecef_pos) {
  return quat_from_rotvec(w_E * t) * ecef_pos;
}
Vec<3> inertial_vel_from_ecef_vel(double t, Vec<3> ecef_vel, Vec<3> eci_pos) {
  return quat_from_rotvec(w_E * t) * ecef_vel +
    w_E.cross(eci_pos);
}
Quaternion inertial_orient_from_ecef_orient(double t, Quaternion ecef_orient) {
  // (eci<-ecef) * (ecef<-body)
  return quat_from_rotvec(w_E * t) * ecef_orient;
}
Vec<3> inertial_acc_from_ecef_acc(double t, Vec<3> ecef_acc, Vec<3> eci_pos) {
  return quat_from_rotvec(w_E * t) * ecef_acc +
    w_E.cross(w_E.cross(eci_pos));
}

Vec<3> ecef_from_inertial(double t, Vec<3> eci_pos) {
  return quat_from_rotvec(-w_E * t) * eci_pos;
}
Vec<3> ecef_vel_from_inertial_vel(double t, Vec<3> eci_vel, Vec<3> eci_pos) {
  return quat_from_rotvec(-w_E * t) *
    (eci_vel - w_E.cross(eci_pos));
}
Quaternion ecef_orient_from_inertial_orient(double t, Quaternion inertial_orient) {
  return quat_from_rotvec(-w_E * t) * inertial_orient;
}
Vec<3> ecef_acc_from_inertial_acc(double t, Vec<3> eci_acc, Vec<3> eci_pos) {
  return quat_from_rotvec(-w_E * t) *
    (eci_acc - w_E.cross(w_E.cross(eci_pos)));
}



SqMat<3> enu_from_ecef_mat(Vec<3> zero_pos_ecef) {
  double x = zero_pos_ecef(0), y = zero_pos_ecef(1), z = zero_pos_ecef(2);
  
  // WGS 84
  double a = 6378137.0;
  double f = 1/298.257223563;

  double e = sqrt(2*f - f*f);
  double b = a*(1 - f);
  
  // Ferrari's solution
  double zeta = (1 - (e*e)) * (z*z) / (a*a);
  double p = sqrt((x*x) + (y*y));
  double rho = ((p*p) / (a*a) + zeta - (e*e*e*e)) / 6;
  double s = (e*e*e*e) * zeta * (p*p) / (4 * (a*a));
  double t = cbrt((rho*rho*rho*rho) + s + sqrt(s * (s + 2*(rho*rho*rho*rho))));
  double u = rho + t + (rho*rho) / t;
  double v = sqrt((u*u) + (e*e*e*e) * zeta);
  double w = (e*e) * (u + v - zeta) / (2 * v);
  double k = 1 + (e*e)*(sqrt(u + v + (w*w)) + w) / (u + v);
  double k0_inv = 1 - (e*e);
  double h = 1/(e*e)*(1/k-k0_inv)*sqrt((p*p) + (z*z)*(k*k));
  double tan_lat = z*k/p;
  
  double sin_lat = tan_lat/sqrt(tan_lat*tan_lat + 1);
  double cos_lat = 1/sqrt(tan_lat*tan_lat + 1);
  
  double cos_lon = x/p;
  double sin_lon = y/p;
  
  Vec<3> up_ecef = (Vec<3>() << cos_lat*cos_lon, cos_lat*sin_lon, sin_lat).finished();
  Vec<3> east_ecef = (Vec<3>() << -sin_lon, cos_lon, 0).finished();
  Vec<3> north_ecef = (Vec<3>() << -sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat).finished();
  return (SqMat<3>() << east_ecef, north_ecef, up_ecef).finished().transpose();
}

}

#endif
