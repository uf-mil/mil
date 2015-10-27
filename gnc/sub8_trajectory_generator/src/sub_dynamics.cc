/**
 * Author: Patrick Emami
 * Date: 10/25/15
 *
 * Description of the ODE governing the motion of the sub
 */

#include "sub_dynamics.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include <cmath>
#include <cassert>

using sub8::trajectory_generator::SubDynamics;

SubDynamics::SubDynamics(const TGenThrusterInfoPtr& ti)
    : _thruster_info_ptr(ti), _mass(25.0){};

// ****************** State representations ********************//
//
// ****************** Memory map for q *************************//
//
// 0: x ------------- position
// 1: y
// 2: z
// 3: x_dot --------- linear velocity
// 4: y_dot
// 5: z_dot
// 6: w_x ----------- angular velocity
// 7: w_y
// 8: w_z
// 9: q_x ----------- orientation
// 10: q_y
// 11: q_z
// 12: q_w
//
// ****************** Memory map for qdot **********************//
//
// 0: x_dot
// 1: y_dot
// 2: z_dot
// 3: x_double_dot -- linear acceleration in x
// 4: y_double_dot -- linear acceleration in y
// 5: z_double_dot -- linear acceleration in z
// 6: w_x_dot ------- angular acceleration in x
// 7: w_y_dot ------- angular acceleration in y
// 8: w_z_dot ------- angular acceleration in z
// 9: q_x_dot ------- rate change of orientation in x
// 10: q_y_dot ------ rate change of orientation in y
// 11: q_z_dot ------ rate change of orientation in z
// 12: q_w_dot ------ rate change of orientation in w
//
// ***************** Dynamics equations *************************//
//
// http://ssl.mit.edu/spheres/library/ASS2011_11-033_spheres.pdf
//
// **************************************************************//
void SubDynamics::ode(const oc::ODESolver::StateType& q_t, const oc::Control* c,
                      oc::ODESolver::StateType& qdot_t) {
  // Ensure qdot is the same size as q, zero out all values
  qdot_t.resize(q_t.size(), 0);

  // Retrieve values, converting to Eigen format
  const double* u_t = c->as<oc::RealVectorControlSpace::ControlType>()->values;

  Vector13d q(q_t.data());
  Vector13d qdot(qdot_t.data());
  Vector8d u(u_t);

  auto Lu = _thruster_info_ptr->L * u;
  auto Du = _thruster_info_ptr->D * u;

  // calculate linear velocities
  qdot.segment(0, 3) = q.segment(3, 3);

  // calculate direction cosine matrix given the orientation
  Matrix3d dcm;
  get_transform(q.segment(9, 4), dcm);

  // calculate linear acceleration
  // 1/m*(R*D*u)
  qdot.segment(3, 3) = (1 / _mass) * dcm * Du;

  // calculate angular acceleration
  // w_dot = inv(I)(-w^x*I*w + Lu)
  //
  // Use the identity matrix temporarily to
  // represent the sub's inertia matrix
  Matrix3d inertia = MatrixXd::Identity(3, 3);
  Vector3d w = q.segment(6, 3);
  Matrix3d w_ss;
  get_skew_symmetric(w, w_ss);
  qdot.segment(6, 3) = inertia.inverse() * (-w_ss * inertia * w + Lu);

  // calculate angular velocities
  // 1/2 *omega(w)*q
  Matrix4d omega;
  omega << -w_ss, w, -w.transpose(), 0;
  qdot.segment(9, 4) = 0.5 * omega * q.segment(9, 4);

  assert(qdot_t.size()==qdot.size()); 
  memcpy(&qdot_t[0], qdot.data(),
         sizeof(double) * qdot.rows() * qdot.cols());
}

void SubDynamics::get_transform(const Vector4d& orientation,
                                Matrix3d& dcm) const {
  // scalar
  double q_w = orientation(3);
  // 3 x 1 column vector
  Vector3d q_hat = orientation.segment(0, 3);
  // 3 x 3 identity
  Matrix3d I = MatrixXd::Identity(3, 3);
  // get skew_symmetric form of q_hat
  Matrix3d q_hat_ss;
  get_skew_symmetric(q_hat, q_hat_ss);

  dcm = (pow(q_w, 2) - q_hat.transpose() * q_hat) * I +
        2 * q_hat * q_hat.transpose() + 2 * q_w * q_hat_ss;
}

void SubDynamics::get_skew_symmetric(const Vector3d& v, Matrix3d& skew) const {
  skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}