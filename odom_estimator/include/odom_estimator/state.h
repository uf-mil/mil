#ifndef GUARD_OPRKZNWIUHZBHYYB
#define GUARD_OPRKZNWIUHZBHYYB

#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include <odom_estimator/util.h>
#include <odom_estimator/gravity.h>
#include <odom_estimator/earth.h>
#include <odom_estimator/unscented_transform.h>
#include <odom_estimator/manifold.h>

namespace odom_estimator {



struct State {
  ros::Time t;
  ros::Time t_start;
  
  static const unsigned int POS_ECI = 0; Vec<3> pos_eci;
  static const unsigned int REL_POS_ECI = POS_ECI + 3; Vec<3> rel_pos_eci;
  static const unsigned int ORIENT = REL_POS_ECI + 3; Quaternion orient;
  static const unsigned int VEL = ORIENT + 3; Vec<3> vel;
  static const unsigned int GYRO_BIAS = VEL + 3; Vec<3> gyro_bias;
  static const unsigned int LOCAL_G = GYRO_BIAS + 3; double local_g;
  static const unsigned int GROUND_AIR_PRESSURE = LOCAL_G + 1; double ground_air_pressure;
  
  std::vector<int> gps_prn;
  static const unsigned int GPS_BIAS = GROUND_AIR_PRESSURE + 1; Vec<Dynamic> gps_bias;
  
  static const int RowsAtCompileTime = Dynamic;
  typedef Vec<RowsAtCompileTime> DeltaType;
  typedef SqMat<RowsAtCompileTime> CovType;
  
  State(ros::Time t, ros::Time t_start,
      Vec<3> pos_eci, Vec<3> rel_pos_eci, Quaternion orient,
      Vec<3> vel, Vec<3> gyro_bias,
      double local_g, double ground_air_pressure,
      std::vector<int> gps_prn, Vec<Dynamic> gps_bias) :
    t(t), t_start(t_start),
    pos_eci(pos_eci), rel_pos_eci(rel_pos_eci),
    orient(orient.normalized()), vel(vel),
    gyro_bias(gyro_bias), local_g(local_g),
    ground_air_pressure(ground_air_pressure),
    gps_prn(gps_prn), gps_bias(gps_bias) {
      assert_none_nan(pos_eci); assert_none_nan(orient.coeffs());
      assert_none_nan(vel); assert_none_nan(gyro_bias);
      assert(std::isfinite(local_g)); assert(std::isfinite(ground_air_pressure));
      assert_none_nan(gps_bias);
      assert(static_cast<int64_t>(gps_bias.rows()) ==
             static_cast<int64_t>(gps_prn.size()));
  }
  unsigned int rows() const {
    return GPS_BIAS + gps_bias.rows();
  }
  
  DeltaType operator-(const State &other) const {
    assert(other.t == t);
    assert(other.t_start == t_start);
    assert(other.gps_prn == gps_prn);
    return (DeltaType(rows()) <<
      pos_eci - other.pos_eci,
      rel_pos_eci - other.rel_pos_eci,
      rotvec_from_quat(orient * other.orient.conjugate()),
      vel - other.vel,
      gyro_bias - other.gyro_bias,
      local_g - other.local_g,
      ground_air_pressure - other.ground_air_pressure,
      gps_bias - other.gps_bias).finished();
  }
  State operator+(const DeltaType &other) const {
    return State(
      t,
      t_start,
      pos_eci + other.segment<3>(POS_ECI),
      rel_pos_eci + other.segment<3>(REL_POS_ECI),
      quat_from_rotvec(other.segment<3>(ORIENT)) * orient,
      vel + other.segment<3>(VEL),
      gyro_bias + other.segment<3>(GYRO_BIAS),
      local_g + other(LOCAL_G),
      ground_air_pressure + other(GROUND_AIR_PRESSURE),
      gps_prn,
      gps_bias + other.tail(other.rows() - GPS_BIAS));
  }
  
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
};


class StateUpdater : public IDistributionFunction<State, State,
  // argh, no way to use a typedef defined within the class for the base class
  // this is ExtraType:
  ManifoldPair<ManifoldPair<Vec<3>, Vec<3> >, Vec<1> >
> {
  typedef ManifoldPair<Vec<3>, Vec<3> > IMUData;
  typedef Vec<1> NoiseType;
  typedef ManifoldPair<IMUData, NoiseType> ExtraType;
  
  sensor_msgs::Imu const imu;
  bool const rightSideAccelFrame;
  
  GaussianDistribution<ExtraType> get_extra_distribution() const {
    return GaussianDistribution<ExtraType>(
      ExtraType(
        IMUData(
          xyz2vec(imu.angular_velocity),
          xyz2vec(imu.linear_acceleration)),
        NoiseType::Zero()),
      ExtraType::build_cov(
        IMUData::build_cov(
          Eigen::Map<const SqMat<3> >(imu.angular_velocity_covariance.data()),
          Eigen::Map<const SqMat<3> >(imu.linear_acceleration_covariance.data())),
        scalar_matrix(5)));
  }
  State apply(State const &state, ExtraType const &extra) const {
    IMUData const &imudata = extra.first;
    NoiseType const &noise = extra.second;
    
    double dt = (imu.header.stamp - state.t).toSec();
    
    Vec<3> angvel_body = imudata.first - state.gyro_bias;
    Quaternion oldbody_from_newbody = quat_from_rotvec(dt * angvel_body);
    
    Quaternion world_from_newbody = state.orient * oldbody_from_newbody;
    
    Vec<3> accelnograv_accelbody = imudata.second;
    Quaternion world_from_accelbody = rightSideAccelFrame ?
      world_from_newbody : state.orient;
    Vec<3> accelnograv_world = world_from_accelbody._transformVector(
      accelnograv_accelbody);
    Vec<3> accel_world = accelnograv_world + gravity::gravity(state.pos_eci);
    
    return State(
      imu.header.stamp,
      state.t_start,
      state.pos_eci + dt * state.vel + dt*dt/2 * accel_world,
      state.rel_pos_eci + dt * state.vel + dt*dt/2 * accel_world,
      world_from_newbody,
      state.vel + dt * accel_world,
      state.gyro_bias,
      state.local_g,
      state.ground_air_pressure + sqrt(dt) * noise(0),
      state.gps_prn,
      state.gps_bias);
  }

public:
  StateUpdater(sensor_msgs::Imu const &imu, bool rightSideAccelFrame=false) :
    imu(imu), rightSideAccelFrame(rightSideAccelFrame) {
  }
};



}

#endif
