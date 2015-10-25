/**
 * Author: Patrick Emami
 * Date: 10/23/15
 *
 * Maintains information on the sub's thrusters
 */
#ifndef TGEN_THRUSTER_INFO_H_
#define TGEN_THRUSTER_INFO_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include "tgen_common.h"

using namespace Eigen;

namespace sub8 {

namespace trajectory_generator {

// forward declarations for typedef
class TGenThrusterInfo;
// Typedef for shared_ptr wrapper
typedef boost::shared_ptr<TGenThrusterInfo> TGenThrusterInfoPtr;

//////////////////////////////////////////////////////////////
//    Encapsulates information
//    about the thrusters needed by the TGEN
//    for its ODE 
//////////////////////////////////////////////////////////////
struct TGenThrusterInfo {
  // Gets the /busses param and 
  // extracts the position and direction
  // matrices
  TGenThrusterInfo();
  // Thruster lever arms 
  Matrix3_8d L;
  // Thruster directions
  Matrix3_8d D;
};
}
}

#endif /* TGEN_THRUSTER_INFO_H_ */