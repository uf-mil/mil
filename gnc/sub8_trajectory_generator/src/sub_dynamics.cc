/**
 * Author: Patrick Emami
 * Date: 10/25/15
 *
 * Description of the ODE governing the motion of the sub
 */

#include "sub_dynamics.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

using sub8::trajectory_generator::SubDynamics;

SubDynamics::SubDynamics(const TGenThrusterInfoPtr& ti)
    : _thruster_info_ptr(ti){};

// ****************** State representations ********************//
//
// ****************** Memory map for q *************************//
//
// 0: x ------------- position
// 1: y
// 2: z
// 3: x_dot --------- linear vel
// 4: y_dot
// 5: z_dot
// 6: w_x ----------- angular vel
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
// 6: w_x_dot ------- alpha_x angular acceleration in x
// 7: w_y_dot ------- alpha_y angular acceleration in y
// 8: w_z_dot ------- alpha_z angular acceleration in z
// 9: q_x_dot
// 10: q_y_dot
// 11: q_z_dot
// 12: q_w_dot
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

  // linear velocities
  qdot(0) = q(3);
  qdot(1) = q(4);
  qdot(2) = q(5);

  auto Lu = _thruster_info_ptr->L * u;
  auto Du = _thruster_info_ptr->D * u; 
  // linear acceleration
  // 1/m(R*D*u)

  // angular acceleration
  // w_dot = alpha = inv(I)(-w^x*I*w + Lu)

  // angular velocities
  // 1/2 *omega(w)*q
}