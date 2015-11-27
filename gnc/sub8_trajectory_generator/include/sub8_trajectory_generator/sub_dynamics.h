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
namespace ob = ompl::base;

namespace sub8 {

namespace trajectory_generator {

class SubDynamics;
typedef boost::shared_ptr<SubDynamics> SubDynamicsPtr;

// The class encapsulates all information
// necessary for carrying out the sub's ODE
// calculations, along with the ode function
// that is passed to the OMPL StatePropagator
class SubDynamics {
 public:
  SubDynamics(TGenThrusterInfoPtr& ti);
  void ode(const oc::ODESolver::StateType& q_t, const oc::Control* c,
           oc::ODESolver::StateType& qdot_t);
  // re-normalize the orientation to be a unit quaternion
  void postPropagate(const ob::State* state, const oc::Control* control,
                     const double duration, ob::State* result);

 private:
  void getTransform(const Vector4d& orientation, Matrix3d& dcm) const;
  void getSkewSymmetric(const Vector3d& in, Matrix3d& skew) const;
  TGenThrusterInfoPtr _thruster_info_ptr;
  double _mass;
};
}
}
#endif /* SUB_DYNAMICS_H_ */