/**
 * Author: Patrick Emami
 * Date: 9/28/15
 *
 * Description of the ODE governing the motion of the sub
 */
#ifndef SUB8_ODE_SOLVER_H_
#define SUB8_ODE_SOLVER_H_

#include "ompl/control/ODESolver.h"
#include "sub8_tgen_common.h"
#include <Eigen/Dense>

using namespace Eigen;

namespace oc = ompl::control;

namespace sub8 {

namespace trajectory_generator {

// ****************** State representations ********************//
//
// ****************** Memory map for q *************************//
//
// 0: x            -- position
// 1: y
// 2: z
// 3: x_dot        -- linear vel
// 4: y_dot
// 5: z_dot
// 6: w_x          -- angular vel
// 7: w_y
// 8: w_z
// 9: q_x          -- orientation (not to be confused with the state q)
// 10: q_y
// 11: q_z
// 12: q_w
//
// ****************** Memory map for qdot **********************//
//
// 0: x_dot
// 1: y_dot
// 2: z_dot
// 3: x_double_dot -- (linear acceleration in x)
// 4: y_double_dot -- (linear acceleration in y)
// 5: z_double_dot -- (linear acceleration in z)
// 6: w_x_dot      -- (alpha_x) angular acceleration in x
// 7: w_y_dot      -- (alpha_y) angular acceleration in y
// 8: w_z_dot      -- (alpha_z) angular acceleration in z
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
void sub8ODE(const oc::ODESolver::StateType& q_t, const oc::Control* c,
             oc::ODESolver::StateType& qdot_t) {
  // Ensure qdot is the same size as q, zero out all values
  qdot_t.resize(q_t.size(), 0);

  // Retrieve control values
  const double* u_t =
      c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

  // Create an Eigen representation without copying over values
  Map<const VectorCd> u(u_t);

  // linear velocities
  qdot[0] = q[3];
  qdot[1] = q[4];
  qdot[2] = q[5];

  // linear acceleration
  // 1/m(R*D*u)

  // angular acceleration
  // w_dot = alpha = inv(I)(-w^x*I*w + Lu)

  // angular velocities
  // 1/2 *omega(w)*q
}
}
}
#endif /* SUB8_ODE_SOLVER */