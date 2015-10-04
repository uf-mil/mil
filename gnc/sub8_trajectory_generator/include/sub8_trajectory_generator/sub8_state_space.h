/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#ifndef SUB8_STATE_SPACE_H_
#define SUB8_STATE_SPACE_H_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

using ompl::base::CompoundStateSpace;
using ompl::base::RealVectorStateSpace;
using ompl::base::RealVectorBounds;
using ompl::base::StateSpacePtr;
using ompl::base::State;

namespace sub8 {

namespace trajectory_generator {

// Defines a custom state space, derived from
// ompl::base::CompoundStateSpace.
//
// This state space consists of [q, qdot], where
// [q] is defined as position and orientation in the world coordinate frame
// and [qdot] is the time rate of change of [q].
// See sub8_ode_solver.h for a description of the dynamics of the system
class Sub8StateSpace : public CompoundStateSpace {
 public:
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() : CompoundStateSpace::StateType() {}

    // The as<SomeStateSpace::StateType>(i) call returns a reference
    // to the i'th subspace of the CompoundStateSpace, of type "SomeStateSpace"

    // Set the x component of the position
    void setX(double x) const {
      as<RealVectorStateSpace::StateType>(0)->values[0] = x;
    }

    // Set the y component of the position
    void setY(double y) const {
      as<RealVectorStateSpace::StateType>(0)->values[1] = y;
    }

    // set the z component of the position
    void setZ(double z) const {
      as<RealVectorStateSpace::StateType>(0)->values[2] = z;
    }

    // Set the linear velocity in the x direction
    void setXDot(double xdot) const {
      as<RealVectorStateSpace::StateType>(1)->values[0] = xdot;
    }

    // Set the linear velocity in the y direction
    void setYDot(double ydot) const {
      as<RealVectorStateSpace::StateType>(1)->values[1] = ydot;
    }

    // Set the linear velocity in the z direction
    void setZDot(double zdot) const {
      as<RealVectorStateSpace::StateType>(1)->values[2] = zdot;
    }

    // Set the angular velocity about the x axis
    void setWx(double wx) const {
      as<RealVectorStateSpace::StateType>(2)->values[0] = wx;
    }

    // Set the angular velocity about the y axis
    void setWy(double wy) const {
      as<RealVectorStateSpace::StateType>(2)->values[1] = wy;
    }

    // Set the angular velocity about the z axis
    void setWz(double wz) const {
      as<RealVectorStateSpace::StateType>(2)->values[2] = wz;
    }

    // Set q_x from quaternion
    void setQx(double qx) const {
      as<RealVectorStateSpace::StateType>(3)->values[0] = qx;
    }

    // Set q_y from quaternion
    void setQy(double qy) const {
      as<RealVectorStateSpace::StateType>(3)->values[1] = qy;
    }

    // Set q_z from quaternion
    void setQz(double qz) const {
      as<RealVectorStateSpace::StateType>(3)->values[2] = qz;
    }

    // Set q_w from quaternion
    void setQw(double qw) const {
      as<RealVectorStateSpace::StateType>(3)->values[3] = qw;
    }
  };
  // Subspaces:
  // 1. RealVectorStateSpace(3) - position
  // 2. RealVectorStateSpace(3) - linear velocities
  // 3. RealVectorStateSpace(3) - angular velocities
  // 4. RealVectorStateSpace(4) - unit quaternion representing orientation
  Sub8StateSpace() : CompoundStateSpace() {
    setName("Sub8StateSpace");
    type_ = SUB8_STATE_SPACE_ID;

    // the 1.0 arg refers to the distance fn weighting. Since we are using a
    // custom
    // distance function, these values are all set to 1.0
    addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
    addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
    addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
    addSubspace(StateSpacePtr(new RealVectorStateSpace(4)), 1.0);
    // Prevent clients from modifying the state space further
    lock();
  }

  virtual ~Sub8StateSpace() {}

  /////////////////////////////////////////////////
  /// Getters and Setters for bounds on dynamics
  /////////////////////////////////////////////////

  // Set bounds for the position subspace
  void set_volume_bounds(const RealVectorBounds& bounds) {
    as<RealVectorStateSpace>(0)->setBounds(bounds);
  }

  // Set bounds for the linear velocities
  void set_linear_velocity_bounds(const RealVectorBounds& bounds) {
    as<RealVectorStateSpace>(1)->setBounds(bounds);
  }

  // Set bounds for the angular velocities
  void set_angular_velocity_bounds(const RealVectorBounds& bounds) {
    as<RealVectorStateSpace>(2)->setBounds(bounds);
  }

  const RealVectorBounds& get_volume_bounds() const {
    return as<RealVectorStateSpace>(0)->getBounds();
  }

  const RealVectorBounds& get_linear_velocity_bounds() const {
    return as<RealVectorStateSpace>(1)->getBounds();
  }

  const RealVectorBounds& get_angular_velocity_bounds() const {
    return as<RealVectorStateSpace>(2)->getBounds();
  }

  //////////////////////////////////////////////////
  // inherited methods that need to be implemented
  //////////////////////////////////////////////////

  // Allocates space for each of the CompoundStateSpace's
  // subspaces
  virtual State* allocState() const;

  // Frees the space allocated to this CompoundStateSpace
  // and all its subspaces
  virtual void freeState(State* state) const;

  // Override the default distance metric, which is to use the L2 norm
  // Instead, LQR  will be used to calculate a cost in the control space
  // between two states
  virtual double distance(const State* state1, const State* state2) const;

  // Get the maximum value a call to distance() can return (or an upper bound).
  // For unbounded state spaces, this function can return infinity.
  //
  // Tight upper bounds are preferred because the value of the extent is used
  // in the automatic computation of parameters for planning. If the bounds
  // are less tight, the automatically computed parameters will be less useful.
  virtual double getMaximumExtent() const;

  // Define equality between states
  virtual bool equalStates(const State* state1, const State* state2) const;

 private:
  const static int SUB8_STATE_SPACE_ID = 99;
};
}
}
#endif /* SUB8_STATE_SPACE */
