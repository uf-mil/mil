/**
 * Author: Patrick Emami
 * Date: 10/23/15
 *
 * Maintains information on the sub's thrusters
 */
#ifndef TGEN_THRUSTER_INFO_H_
#define TGEN_THRUSTER_INFO_H_

#include "tgen_common.h"
#include <ros/ros.h>  // boost::shared_ptr

namespace sub8 {

namespace trajectory_generator {

class TGenThrusterInfo;
typedef boost::shared_ptr<TGenThrusterInfo> TGenThrusterInfoPtr;

//////////////////////////////////////////////////////////////
//    Encapsulates information
//    about the thrusters needed by the TGEN
//    for its ODE
//////////////////////////////////////////////////////////////
struct TGenThrusterInfo {
  // Convert a row-major order array containing the
  // of the B matrix to the L and D matrix reps
  void init(std::vector<double>& b) {
    
    unsigned int i = 0;  // thruster idx

    for (std::vector<double>::iterator it = b.begin(); (it + 5) <= b.end();
         it += 6) {
      // i:i+2 - Level arms for thruster i
      // i+3:i+5 - Directions for thruster i
      L(0, i) = (*it);
      L(1, i) = (*(it + 1));
      L(2, i) = (*(it + 2));

      D(0, i) = (*(it + 3));
      D(1, i) = (*(it + 4));
      D(2, i) = (*(it + 5));
      i++;
    }
  }

  // Thruster lever arms
  Matrix3_8d L;
  // Thruster directions
  Matrix3_8d D;
};
}
}
#endif /* TGEN_THRUSTER_INFO_H_ */