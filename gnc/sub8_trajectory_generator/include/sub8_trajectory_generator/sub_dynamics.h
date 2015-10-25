/**
 * Author: Patrick Emami
 * Date: 9/28/15
 *
 * Description of the ODE governing the motion of the sub
 */
#ifndef SUB_DYNAMICS_H_
#define SUB_DYNAMICS_H_

#include "ompl/control/ODESolver.h"
#include "tgen_common.h"
#include "tgen_thruster_info.h"
#include <Eigen/Dense>

using namespace Eigen;
using sub8::trajectory_generator::TGenThrusterInfoPtr;

namespace oc = ompl::control;

namespace sub8 {

namespace trajectory_generator {

class SubDynamics;
typedef boost::shared_ptr<SubDynamics> SubDynamicsPtr;

class SubDynamics {
 public:
  SubDynamics(const TGenThrusterInfoPtr& ti);
  void ode(const oc::ODESolver::StateType& q_t, const oc::Control* c,
               oc::ODESolver::StateType& qdot_t);

 private:
  TGenThrusterInfoPtr _thruster_info_ptr;
};
}
}
#endif /* SUB_DYNAMICS_H_ */