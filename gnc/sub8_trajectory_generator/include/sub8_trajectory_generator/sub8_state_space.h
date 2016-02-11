/**
* Author: Patrick Emami
* Date: 1/27/15
*/
#ifndef SUB8_STATE_SPACE_H_
#define SUB8_STATE_SPACE_H_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/SO2StateSpace.h"

using ompl::base::CompoundStateSpace;
using ompl::base::RealVectorStateSpace;
using ompl::base::RealVectorBounds;
using ompl::base::SO2StateSpace; 

namespace sub8 {
namespace trajectory_generator {

// A state space representing translation in 3D + rotation in the plane
class Sub8StateSpace : public CompoundStateSpace {
 public:
  // A state: (x, y, z, yaw)
  // Units - (meters, radians)
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() : CompoundStateSpace::StateType() {}

    //  Get the X component of the state
    double getX() const {
      return as<RealVectorStateSpace::StateType>(0)->values[0];
    }

    //  Get the Y component of the state
    double getY() const {
      return as<RealVectorStateSpace::StateType>(0)->values[1];
    }

    //  Get the Z component of the state
    double getZ() const {
      return as<RealVectorStateSpace::StateType>(0)->values[2];
    }

    //  Get the yaw component of the state. This is
    //  the rotation in plane, with respect to the Z
    //  axis.
    double getYaw() const { return as<SO2StateSpace::StateType>(1)->value; }

    //  Set the X component of the state */
    void setX(double x) {
      as<RealVectorStateSpace::StateType>(0)->values[0] = x;
    }

    //  Set the Y component of the state */
    void setY(double y) {
      as<RealVectorStateSpace::StateType>(0)->values[1] = y;
    }

    //  Set the Z component of the state */
    void setZ(double z) {
      as<RealVectorStateSpace::StateType>(0)->values[2] = z;
    }

    //  Set the X, Y, and Z components of the state */
    void setXYZ(double x, double y, double z) {
      setX(x);
      setY(y);
      setZ(z);
    }

    //  Set the yaw component of the state. This is
    //  the rotation in plane, with respect to the Z
    //  axis.
    void setYaw(double yaw) { as<SO2StateSpace::StateType>(1)->value = yaw; }
  };

  Sub8StateSpace() : CompoundStateSpace() {
    setName("Sub8StateSpace");
    addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
    addSubspace(StateSpacePtr(new SO2StateSpace()), 0.5);
    lock();
  }

  virtual ~Sub8StateSpace() {}

  // RealVectorStateSpace::setBounds() */
  void setBounds(const RealVectorBounds& bounds) {
    as<RealVectorStateSpace>(0)->setBounds(bounds);
  }

  // RealVectorStateSpace::getBounds() */
  const RealVectorBounds& getBounds() const {
    return as<RealVectorStateSpace>(0)->getBounds();
  }

  State* allocState() const override {
    StateType* state = new StateType();
    allocStateComponents(state);
    return state;
  }

  void freeState(State* state) const override {
    CompoundStateSpace::freeState(state);
  }
};
}
}

#endif