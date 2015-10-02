/**
 * Author: Patrick Emami
 * Date: 9/28/15
 *
 * Description of the ODE governing the motion of the sub
 */
#ifndef SUB8_ODE_SOLVER_H_
#define SUB8_ODE_SOLVER_H_

#include "ompl/control/ODESolver.h"

namespace oc = ompl::control;

namespace sub8 {

namespace trajectory_generator {

void sub8ODE(const oc::ODESolver::StateType& q, const oc::Control* c,
             oc::ODESolver::StateType& qdot) {

  // Retrieve control values
  const double* u =
      c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  
  // The memory for Sub8StateSpace is mapped as: 
  // 0: x
  // 1: y
  // 2: z
  // 3: x_dot
  // 4: y_dot
  // 5: z_dot
  // 6: w_x
  // 7: w_y
  // 8: w_z
  // 9: rotation[0]
  // 10: rotation[1]
  // 11: rotation[2]

  // Ensure qdot is the same size as q, zero out all values
  qdot.resize(q.size(), 0); 

  // qdot memory mapping
  // 0: x_dot
  // 1: y_dot
  // 2: z_dot
  // 3: x_double_dot
  // 4: y_double_dot
  // 5: z_double_dot
  // 6: w_x_dot
  // 7: w_y_dot
  // 8: w_z_dot
  // 9: w_x
  // 10: w_y
  // 11: w_z

  // linear velocities
  qdot[0] = q[3];
  qdot[1] = q[4]; 
  qdot[2] = q[5]; 

  // angular velocities
  qdot[9] = q[6]; 
  qdot[10] = q[7]; 
  qdot[11] = q[8]; 

  // linear acceleration?

  // Fx + Fy + Fz / m

  // angular acceleration? 

  // inverse of inertia matrix 
}
}
}
#endif /* SUB8_ODE_SOLVER */