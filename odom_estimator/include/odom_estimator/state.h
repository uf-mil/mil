#ifndef GUARD_OPRKZNWIUHZBHYYB
#define GUARD_OPRKZNWIUHZBHYYB

#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "odom_estimator/util.h"
#include "odom_estimator/gravity.h"
#include "odom_estimator/earth.h"
#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/manifold.h"

namespace odom_estimator {


ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(State,
  (ros::Time, t)
  (ros::Time, t_start)
  (std::vector<int>, gps_prn), // XXX assert size == gps_bias.rows() somehow
  
  (Vec<3>, pos_eci)
  (Vec<3>, rel_pos_eci)
  (QuaternionManifold, orient)
  (Vec<3>, vel)
  (Vec<3>, gyro_bias)
  (Vec<3>, accel_bias)
  (WrappedScalar, ground_air_pressure)
  (Vec<Dynamic>, gps_bias)
)
  Vec<3> getPosECI(Vec<3> body_point=Vec<3>::Zero()) const {
    return pos_eci + orient._transformVector(body_point);
  }
  Vec<3> getPosECEF(Vec<3> body_point=Vec<3>::Zero()) const {
    return ecef_from_inertial(t.toSec(), getPosECI(body_point));
  }
  Vec<3> getRelPosECEF() const {
    return ecef_from_inertial(t.toSec(), pos_eci) -
      ecef_from_inertial(t_start.toSec(), pos_eci - rel_pos_eci);
  }
  Vec<3> getVelECI(Vec<3> body_point=Vec<3>::Zero(),
                   boost::optional<Vec<3> > gyro=boost::none) const {
    Vec<3> result = vel;
    if(body_point != Vec<3>::Zero()) {
      assert(gyro);
      result += orient._transformVector(
        (*gyro - gyro_bias).cross(body_point));
    }
    return result;
  }
  Vec<3> getVelECEF(Vec<3> body_point=Vec<3>::Zero(),
                   boost::optional<Vec<3> > gyro=boost::none) const {
    return ecef_vel_from_inertial_vel(t.toSec(),
      getVelECI(body_point, gyro),
      pos_eci);
  }
  Quaternion getOrientECEF() const {
    return ecef_orient_from_inertial_orient(t.toSec(), orient);
  }
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()


ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(_PredictNoise, ,
  (Vec<3>, gyro)
  (Vec<3>, accel)
  (WrappedScalar, ground_air_pressure_noise)
  (Vec<3>, gyro_bias_noise)
  (Vec<3>, accel_bias_noise)
)
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()
class StateUpdater : public UnscentedTransformDistributionFunction<State, State, _PredictNoise> {
  sensor_msgs::Imu const imu;
  bool const rightSideAccelFrame;
  
  GaussianDistribution<_PredictNoise> get_extra_distribution() const {
    return GaussianDistribution<_PredictNoise>(
      _PredictNoise(
        xyz2vec(imu.angular_velocity),
        xyz2vec(imu.linear_acceleration),
        0,
        Vec<3>::Zero(),
        Vec<3>::Zero()),
      joinDiagonally(
        joinDiagonally(
          Eigen::Map<const SqMat<3> >(imu.angular_velocity_covariance.data()),
          Eigen::Map<const SqMat<3> >(imu.linear_acceleration_covariance.data())),
        joinDiagonally(
          scalar_matrix(5),
          joinDiagonally(
            (Vec<3>::Ones()*pow(1e-3, 2)).asDiagonal(),
            (Vec<3>::Ones()*pow(1e-3, 2)).asDiagonal()))));
  }
  State apply(State const &state, _PredictNoise const &extra) const {
    double dt = (imu.header.stamp - state.t).toSec();
    
    Vec<3> angvel_body = extra.gyro - state.gyro_bias;
    Quaternion oldbody_from_newbody = quat_from_rotvec(dt * angvel_body);
    
    Quaternion world_from_newbody = state.orient * oldbody_from_newbody;
    
    Vec<3> accelnograv_accelbody = extra.accel - state.accel_bias;
    Quaternion world_from_accelbody = rightSideAccelFrame ?
      world_from_newbody : Quaternion(state.orient);
    Vec<3> accelnograv_world = world_from_accelbody._transformVector(
      accelnograv_accelbody);
    Vec<3> accel_world = accelnograv_world + gravity::gravity(state.pos_eci);
    
    return State(
      imu.header.stamp,
      state.t_start,
      state.gps_prn,
      state.pos_eci + dt * state.vel + dt*dt/2 * accel_world,
      state.rel_pos_eci + dt * state.vel + dt*dt/2 * accel_world,
      world_from_newbody,
      state.vel + dt * accel_world,
      state.gyro_bias + sqrt(dt) * extra.gyro_bias_noise,
      state.accel_bias + sqrt(dt) * extra.accel_bias_noise,
      state.ground_air_pressure + sqrt(dt) * extra.ground_air_pressure_noise,
      state.gps_bias);
  }

public:
  StateUpdater(sensor_msgs::Imu const &imu, bool rightSideAccelFrame=false) :
    imu(imu), rightSideAccelFrame(rightSideAccelFrame) {
  }
};



}

#endif
