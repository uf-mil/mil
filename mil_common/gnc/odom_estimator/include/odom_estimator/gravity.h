#ifndef GUARD_FPPPRRCZKGOFCAVZ
#define GUARD_FPPPRRCZKGOFCAVZ

#include "odom_estimator/util.h"

namespace odom_estimator
{
namespace gravity
{
double a_E = 6378.136e3;
double mu = 398600.44e9;
double C_20 = -1082.63e-6;

Vec<3> gravity(Vec<3> pos)
{
  // returns gravity in ECI frame given an ECI position
  // does not include centrifugal force
  Vec<3> pos_bar = pos.normalized();
  double mu_bar = mu / pow(pos.norm(), 2);
  double rho = a_E / pos.norm();
  return -mu_bar * pos_bar +
         3 / 2. * C_20 * mu_bar * pow(rho, 2) *
             pos_bar.cwiseProduct(
                 Vec<3>(1 - 5 * pow(pos_bar[2], 2), 1 - 5 * pow(pos_bar[2], 2), 3 - 5 * pow(pos_bar[2], 2)));
}
}
}

#endif
