/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#ifndef SUB8_STATE_SPACE_H_
#define SUB8_STATE_SPACE_H_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "tgen_common.h"
#include <Eigen/Dense>

using namespace Eigen;
using ompl::base::CompoundStateSpace;
using ompl::base::RealVectorStateSpace;
using ompl::base::RealVectorBounds;
using ompl::base::StateSpacePtr;
using ompl::base::State;

namespace sub8 {

namespace trajectory_generator {

class Sub8StateSpace;
typedef boost::shared_ptr<Sub8StateSpace> Sub8StateSpacePtr;

// Defines a custom state space, derived from
// ompl::base::CompoundStateSpace.
//
// This state space consists of [q, qdot], where
// [q] is defined as position and orientation in the world coordinate frame
// and [qdot] is the time rate of change of [q].
// See sub_dynamics.h for a description of the dynamics of the system
class Sub8StateSpace : public CompoundStateSpace {
 public:
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() : CompoundStateSpace::StateType() {}

    // The as<SomeStateSpace::StateType>(i) call returns a reference
    // to the i'th subspace of the CompoundStateSpace, of type "SomeStateSpace"

    /////////////////////////////////////////////////
    // Getters and Setters, necessary for
    // sending State obj's to ROS and for
    // converting Waypoint messages to State obj's
    /////////////////////////////////////////////////

    // Set x,y,z position
    void setPosition(double x, double y, double z) const {
      as<RealVectorStateSpace::StateType>(0)->values[0] = x;
      as<RealVectorStateSpace::StateType>(0)->values[1] = y;
      as<RealVectorStateSpace::StateType>(0)->values[2] = z;
    }

    // pos(0) = x
    // pos(1) = y
    // pos(2) = z
    void getPosition(Vector3d& pos) const {
      pos << as<RealVectorStateSpace::StateType>(0)->values[0],
          as<RealVectorStateSpace::StateType>(0)->values[1],
          as<RealVectorStateSpace::StateType>(0)->values[2];
    }

    // Set x,y,z velocities
    void setLinearVelocity(double xdot, double ydot, double zdot) const {
      as<RealVectorStateSpace::StateType>(1)->values[0] = xdot;
      as<RealVectorStateSpace::StateType>(1)->values[1] = ydot;
      as<RealVectorStateSpace::StateType>(1)->values[2] = zdot;
    }

    // vel(0) = x
    // vel(1) = y
    // vel(2) = z
    void getLinearVelocity(Vector3d& vel) const {
      vel << as<RealVectorStateSpace::StateType>(1)->values[0],
          as<RealVectorStateSpace::StateType>(1)->values[1],
          as<RealVectorStateSpace::StateType>(1)->values[2];
    }

    // Set wx, wy, wz angular velocities
    void setAngularVelocity(double wx, double wy, double wz) const {
      as<RealVectorStateSpace::StateType>(2)->values[0] = wx;
      as<RealVectorStateSpace::StateType>(2)->values[1] = wy;
      as<RealVectorStateSpace::StateType>(2)->values[2] = wz;
    }

    // w(0) = wx
    // w(1) = wy
    // w(2) = wz
    void getAngularVelocity(Vector3d& w) const {
      w << as<RealVectorStateSpace::StateType>(2)->values[0],
          as<RealVectorStateSpace::StateType>(2)->values[1],
          as<RealVectorStateSpace::StateType>(2)->values[2];
    }

    // Set qx, qy, qz, qw for the unit quaternion representing orientation
    void setOrientation(double qx, double qy, double qz, double qw) const {
      as<RealVectorStateSpace::StateType>(3)->values[0] = qx;
      as<RealVectorStateSpace::StateType>(3)->values[1] = qy;
      as<RealVectorStateSpace::StateType>(3)->values[2] = qz;
      as<RealVectorStateSpace::StateType>(3)->values[3] = qw;
    }

    // orientation[0] = qx
    // orientation[1] = qy
    // orientation[2] = qz
    // orientation[3] = qw
    void getOrientation(Vector4d& orientation) const {
      orientation << as<RealVectorStateSpace::StateType>(3)->values[0],
          as<RealVectorStateSpace::StateType>(3)->values[1],
          as<RealVectorStateSpace::StateType>(3)->values[2],
          as<RealVectorStateSpace::StateType>(3)->values[3];
    }

    void getState(Vector13d& state) const {
      Vector3d position;
      getPosition(position);
      Vector3d linear_velocity;
      getLinearVelocity(linear_velocity);
      Vector3d angular_velocity;
      getAngularVelocity(angular_velocity);
      Vector4d orientation;
      getOrientation(orientation);
      state << position, linear_velocity, angular_velocity, orientation;
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

  // Set bounds on the orientation (despite the fact that the
  // sub is holonomic, since we're using a 4D RealVectorSpace
  // for its orientation, OMPL requires us to set bounds)
  void set_orientation_bounds(const RealVectorBounds& bounds) {
    as<RealVectorStateSpace>(3)->setBounds(bounds);
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

  const RealVectorBounds& get_orientation_bounds() const {
    return as<RealVectorStateSpace>(3)->getBounds();
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
