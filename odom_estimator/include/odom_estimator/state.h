#ifndef GUARD_OPRKZNWIUHZBHYYB
#define GUARD_OPRKZNWIUHZBHYYB

#include <ros/time.h>

#include <Eigen/Dense>

#include <odom_estimator/kalman.h>

namespace odom_estimator {



using namespace Eigen;

struct State {
  ros::Time t;
  
  static const unsigned int POS = 0; Vector3d pos;
  static const unsigned int ORIENT = POS + 3; Quaterniond orient;
  static const unsigned int VEL = ORIENT + 3; Vector3d vel;
  static const unsigned int GYRO_BIAS = VEL + 3; Vector3d gyro_bias;
  static const unsigned int LOCAL_G = GYRO_BIAS + 3; double local_g;
  static const unsigned int GROUND_AIR_PRESSURE = LOCAL_G + 1; double ground_air_pressure;
  static const int RowsAtCompileTime = GROUND_AIR_PRESSURE + 1;
  typedef Matrix<double, RowsAtCompileTime, 1> DeltaType;
  typedef Matrix<double, RowsAtCompileTime, RowsAtCompileTime> CovType;
  
  State(ros::Time t, Vector3d pos, Quaterniond orient,
      Vector3d vel, Vector3d gyro_bias,
      double local_g, double ground_air_pressure) :
    t(t), pos(pos), orient(orient.normalized()), vel(vel),
    gyro_bias(gyro_bias), local_g(local_g),
    ground_air_pressure(ground_air_pressure) {
      assert_none_nan(pos); assert_none_nan(orient.coeffs());
      assert_none_nan(vel); assert_none_nan(gyro_bias);
      assert(std::isfinite(local_g)); assert(std::isfinite(ground_air_pressure));
  }
  unsigned int rows() const {
    return RowsAtCompileTime;
  }
  
  DeltaType operator-(const State &other) const {
    return (DeltaType() <<
      pos - other.pos,
      rotvec_from_quat(orient * other.orient.conjugate()),
      vel - other.vel,
      gyro_bias - other.gyro_bias,
      local_g - other.local_g,
      ground_air_pressure - other.ground_air_pressure).finished();
  }
  State operator+(const DeltaType &other) const {
    return State(
      t,
      pos + other.segment<3>(POS),
      quat_from_rotvec(other.segment<3>(ORIENT)) * orient,
      vel + other.segment<3>(VEL),
      gyro_bias + other.segment<3>(GYRO_BIAS),
      local_g + other(LOCAL_G),
      ground_air_pressure + other(GROUND_AIR_PRESSURE));
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
  
  GaussianDistribution<ExtraType> get_extra() const {
    return GaussianDistribution<ExtraType>(
      ExtraType(
        IMUData(
          xyz2vec(imu.angular_velocity),
          xyz2vec(imu.linear_acceleration)),
        NoiseType::Zero()),
      ExtraType::build_cov(
        IMUData::build_cov(
          Map<const Matrix3d>(imu.angular_velocity_covariance.data()),
          Map<const Matrix3d>(imu.linear_acceleration_covariance.data())),
        scalar_matrix(5)));
  }
  State apply(State const &state, ExtraType const &extra) const {
    IMUData const &imudata = extra.first;
    NoiseType const &noise = extra.second;
    
    double dt = (imu.header.stamp - state.t).toSec();
    
    Vector3d angvel_body = imudata.first - state.gyro_bias;
    Quaterniond oldbody_from_newbody = quat_from_rotvec(dt * angvel_body);
    
    Quaterniond world_from_newbody = state.orient * oldbody_from_newbody;
    
    Vector3d accelnograv_accelbody = imudata.second;
    Quaterniond world_from_accelbody = rightSideAccelFrame ?
      world_from_newbody : state.orient;
    Vector3d accelnograv_world = world_from_accelbody._transformVector(
      accelnograv_accelbody);
    Vector3d accel_world = accelnograv_world + Vector3d(0, 0, -state.local_g);
    
    return State(
      imu.header.stamp,
      state.pos + dt * state.vel + dt*dt/2 * accel_world,
      world_from_newbody,
      state.vel + dt * accel_world,
      state.gyro_bias,
      state.local_g,
      state.ground_air_pressure + sqrt(dt) * noise(0));
  }

public:
  StateUpdater(sensor_msgs::Imu const &imu, bool rightSideAccelFrame=false) :
    imu(imu), rightSideAccelFrame(rightSideAccelFrame) {
  }
};



}

#endif
