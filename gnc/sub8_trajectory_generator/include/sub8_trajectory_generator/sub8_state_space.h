/**
 * Author: Patrick Emami
 * Date: 9/21/15
 */
#ifndef SUB8_STATE_SPACE_H_
#define SUB8_STATE_SPACE_H_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

using ompl::base::CompoundStateSpace; 
using ompl::base::RealVectorStateSpace; 
using ompl::base::RealVectorBounds;
using ompl::base::SO3StateSpace; 
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
		// The dynamics are modeled with xdot = Ax + Bu, where x = [q, qdot]
		// and xdot = [qdot, qdoubledot]
		class Sub8StateSpace : public CompoundStateSpace {
		public: 
			class StateType : public CompoundStateSpace::StateType {
			public: 
				StateType() : CompoundStateSpace::StateType() {}

				// The as<SomeStateSpace::StateType>(i) call returns a reference 
				// to the i'th subspace of the SompoundStateSpace "SomeStateSpace"

				// Get the x component of the position
				double x() const {
					return as<RealVectorStateSpace::StateType>(0)->values[0]; 
				}

				// Get the y component of the position
				double y() const {
					return as<RealVectorStateSpace::StateType>(0)->values[1]; 
				}

				// Get the z component of the position
				double z() const {
					return as<RealVectorStateSpace::StateType>(0)->values[2]; 
				}

				// Get the linear velocity in the x direction 
				double x_dot() const {
					return as<RealVectorStateSpace::StateType>(1)->values[0]; 
				}

				// Get the linear velocity in the y direction
				double y_dot() const {
					return as<RealVectorStateSpace::StateType>(1)->values[1]; 
				}

				// Get the linear velocity in the z direction
				double z_dot() const {
					return as<RealVectorStateSpace::StateType>(1)->values[2]; 
				}

				// Get the angular velocity about the x axis
				double alpha_dot() const {
					return as<RealVectorStateSpace::StateType>(2)->values[0]; 
				}

				// Get the angular velocity about the y axis
				double beta_dot() const {
					return as<RealVectorStateSpace::StateType>(2)->values[1]; 
				}

				// Get the angular velocity about the z axis
				double gamma_dot() const {
					return as<RealVectorStateSpace::StateType>(2)->values[2]; 
				}

				// Get a const reference to the orientation as a unit quaternion
				const SO3StateSpace::StateType& rotation() const {
					return *as<SO3StateSpace::StateType>(3);
				}

				// Get a reference to the orientation as a unit quaternion
				SO3StateSpace::StateType& rotation() {
					return *as<SO3StateSpace::StateType>(3);
				}
			};	

			// Subspaces: 
			// 1. RealVectorStateSpace(3) - position 
			// 2. RealVectorStateSpace(3) - linear velocities
			// 3. RealVectorStateSpace(3) - angular velocities 
			// 4. SO(3) StateSpace - orientation
			Sub8StateSpace() : CompoundStateSpace() {
				setName("Sub8StateSpace"); 
				type_ = SUB8_STATE_SPACE_ID;

				// the 1.0 arg refers to the distance fn weighting. Since we are using a custom 
				// distance function, these values are all set to 1.0
				addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
				addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
				addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
				addSubspace(StateSpacePtr(new SO3StateSpace()), 1.0);
				// Prevent clients from modifying the state space further
				lock(); 
			}

			virtual ~Sub8StateSpace() {
			}

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
			virtual double distance(const State* state1, const State* state2 ) const; 

			// Get the maximum value a call to distance() can return (or an upper bound).
			// For unbounded state spaces, this function can return infinity.
			//
    		// Tight upper bounds are preferred because the value of the extent is used
    		// in the automatic computation of parameters for planning. If the bounds
    		// are less tight, the automatically computed parameters will be less useful. 
			virtual double getMaximumExtent() const; 

			// Define equality between states
			virtual bool equalStates(const State* state1, const State* state2 ) const; 

		private: 
			const static int SUB8_STATE_SPACE_ID = 99; 
		};
	}
} 	 	
#endif /* SUB8_STATE_SPACE */