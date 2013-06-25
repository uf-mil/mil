#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
//#include <sensor_msgs/FluidPressure.h>
#include <rawgps_common/Measurements.h>

#include "util.h"

using namespace Eigen;

namespace odom_estimator {

struct State {
  ros::Time t;
  
  Vector3d pos;
  Quaterniond orient;
  Vector3d vel;
  Vector3d gyro_bias;
  double local_g;
  double ground_air_pressure;
  static const int STATE_LENGTH = 3*4 + 2;
  typedef Matrix<double, STATE_LENGTH, 1> StateType;
  
  State(ros::Time t, Vector3d pos, Quaterniond orient,
      Vector3d vel, Vector3d gyro_bias,
      double local_g, double ground_air_pressure) :
    t(t), pos(pos), orient(orient.normalized()), vel(vel),
    gyro_bias(gyro_bias), local_g(local_g),
    ground_air_pressure(ground_air_pressure) { }
  
  static const int PREDICT_EXTRA_NOISE_LENGTH = 1;
  Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, PREDICT_EXTRA_NOISE_LENGTH> get_extra_noise_cov() const {
    return scalar_matrix(5);
  }
  State predict(ros::Time t, Vector3d gyro, Vector3d accel,
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> noise,
      bool rightSideAccelFrame=false) const {
    double dt = (t - this->t).toSec();
    
    Vector3d angvel_body = gyro - gyro_bias;
    Quaterniond oldbody_from_newbody = quat_from_rotvec(dt * angvel_body);
    
    Quaterniond world_from_newbody = orient * oldbody_from_newbody;
    
    Vector3d accelnograv_accelbody = accel;
    Quaterniond world_from_accelbody = rightSideAccelFrame ?
      world_from_newbody : orient;
    Vector3d accelnograv_world = world_from_accelbody._transformVector(
      accelnograv_accelbody);
    Vector3d accel_world = accelnograv_world + Vector3d(0, 0, -local_g);
    
    return State(
      t,
      pos + dt * vel + dt*dt/2 * accel_world,
      world_from_newbody,
      vel + dt * accel_world,
      gyro_bias,
      local_g,
      ground_air_pressure + sqrt(dt) * noise(0));
  }
  
  StateType operator-(const State &other) const {
    return (StateType() <<
      pos - other.pos,
      rotvec_from_quat(orient * other.orient.conjugate()),
      vel - other.vel,
      gyro_bias - other.gyro_bias,
      local_g - other.local_g,
      ground_air_pressure - other.ground_air_pressure).finished();
  }
  State operator+(const StateType &other) const {
    return State(
      t,
      pos + other.segment<3>(0),
      quat_from_rotvec(other.segment<3>(3)) * orient,
      vel + other.segment<3>(6),
      gyro_bias + other.segment<3>(9),
      local_g + other(12),
      ground_air_pressure + other(13));
  }
};


struct AugmentedState : public State {
  typedef Matrix<double, STATE_LENGTH, STATE_LENGTH> CovType;
  CovType cov;
  AugmentedState(const State &state, const CovType &cov) :
    State(state), cov(cov/2 + cov.transpose()/2) { }
  
  AugmentedState predict(const sensor_msgs::Imu &imu) const {
    const unsigned int NOISE_LENGTH = 3*2+PREDICT_EXTRA_NOISE_LENGTH;
    typedef Matrix<double, STATE_LENGTH + NOISE_LENGTH, 1> AugmentedStateType;
    typedef Matrix<double, STATE_LENGTH + NOISE_LENGTH, STATE_LENGTH + NOISE_LENGTH> AugmentedMatrixType;
    
    const State &this_state = *this;
    
    int L = STATE_LENGTH + NOISE_LENGTH;
    double alpha = 1e-3;
    double beta = 2;
    double kappa = 0;
    double lambda = pow(alpha, 2)*(L + kappa) - L;
    
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero();
    Pa.block<STATE_LENGTH, STATE_LENGTH>(0, 0) = cov;
    Pa.block<3, 3>(STATE_LENGTH, STATE_LENGTH) =
      Map<const Matrix3d>(imu.angular_velocity_covariance.data());
    Pa.block<3, 3>(STATE_LENGTH + 3, STATE_LENGTH + 3) =
      Map<const Matrix3d>(imu.linear_acceleration_covariance.data());
    Pa.block<PREDICT_EXTRA_NOISE_LENGTH, PREDICT_EXTRA_NOISE_LENGTH>(STATE_LENGTH + 6, STATE_LENGTH + 6) =
      get_extra_noise_cov();
    
    AugmentedMatrixType sqrtkPa = cholesky<STATE_LENGTH + NOISE_LENGTH>((L + lambda)*Pa);
    
    boost::optional<State> respoints[2*L+1];
    for(int i = 0; i <= 2*L; i++) {
      AugmentedStateType dx;
      if(i == 0) dx = AugmentedStateType::Zero();
      else if(i <= L) dx = sqrtkPa.col(i-1);
      else dx = -sqrtkPa.col(i-L-1);
      
      State x_before = this_state + dx.segment<STATE_LENGTH>(0);
      State x_after = x_before.predict(imu.header.stamp,
        xyz2vec(imu.angular_velocity) + dx.segment<3>(STATE_LENGTH),
        xyz2vec(imu.linear_acceleration) + dx.segment<3>(STATE_LENGTH+3),
        dx.segment<PREDICT_EXTRA_NOISE_LENGTH>(STATE_LENGTH+6));
      respoints[i] = x_after;
    }
    
    StateType res = StateType::Zero();
    for(int i = 0; i <= 2*L; i++) {
      double W_s = i == 0 ? lambda/(L + lambda) : 1./2/(L+lambda);
      res += W_s * (*respoints[i] - *respoints[0]);
    }
    State new_state = *respoints[0] + res;
    
    CovType new_cov = CovType::Zero();
    for(int i = 0; i <= 2*L; i++) {
      double W_c = i == 0 ?
        lambda/(L + lambda) + (1 - pow(alpha, 2) + beta) : 1./2/(L+lambda);
      StateType dx = *respoints[i] - new_state;
      new_cov += W_c * dx * dx.transpose();
    }
    
    return AugmentedState(new_state, new_cov);
  }
  
  template <int N, int NN>
  AugmentedState update(
      const boost::function<Matrix<double, N, 1> (State, Matrix<double, NN, 1>)> &observe,
      const Matrix<double, NN, NN> &noise_cov) const {
    unsigned int realNN = NN != Dynamic ? NN : noise_cov.rows();
    assert(noise_cov.rows() == noise_cov.cols());
    
    typedef Matrix<double, NN != Dynamic ? STATE_LENGTH + NN : Dynamic, 1> AugmentedStateType;
    typedef Matrix<double, NN != Dynamic ? STATE_LENGTH + NN : Dynamic, NN != Dynamic ? STATE_LENGTH + NN : Dynamic> AugmentedMatrixType;
    
    const State &this_state = *this;
    
    int L = STATE_LENGTH + realNN;
    double alpha = 1e-3;
    double beta = 2;
    double kappa = 0;
    double lambda = pow(alpha, 2)*(L + kappa) - L;
    
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero(STATE_LENGTH + realNN, STATE_LENGTH + realNN);
    Pa.template block<STATE_LENGTH, STATE_LENGTH>(0, 0) = cov;
    Pa.template bottomRightCorner(realNN, realNN) = noise_cov;
    
    AugmentedMatrixType sqrtkPa = cholesky<NN != Dynamic ? STATE_LENGTH + NN : Dynamic>((L + lambda)*Pa);
    
    StateType sigmapoints[2*L+1];
    Matrix<double, N, 1> errors[2*L+1];
    for(int i = 0; i <= 2*L; i++) {
      AugmentedStateType dx;
      if(i == 0) dx = AugmentedStateType::Zero(STATE_LENGTH + realNN);
      else if(i <= L) dx = sqrtkPa.col(i-1);
      else dx = -sqrtkPa.col(i-L-1);
      
      sigmapoints[i] = dx.head(STATE_LENGTH);
      errors[i] = observe(this_state + dx.head(STATE_LENGTH), dx.tail(realNN));
    }
    
    unsigned int realN = N != Dynamic ? N : errors[0].rows();
    Matrix<double, N, 1> y = Matrix<double, N, 1>::Zero(realN, 1);
    for(int i = 0; i <= 2*L; i++) {
      double W_s = i == 0 ? lambda/(L + lambda) : 1./2/(L+lambda);
      y += W_s * errors[i];
    }
    
    Matrix<double, N, N> P_zz = Matrix<double, N, N>::Zero(realN, realN);
    Matrix<double, STATE_LENGTH, N> P_xz = Matrix<double, STATE_LENGTH, N>::Zero(STATE_LENGTH, realN);
    for(int i = 0; i <= 2*L; i++) {
      double W_c = i == 0 ?
        lambda/(L + lambda) + (1 - pow(alpha, 2) + beta) : 1./2/(L+lambda);
      Matrix<double, N, 1> dy = -errors[i] + y;
      P_zz += W_c * dy * dy.transpose();
      P_xz += W_c * sigmapoints[i] * dy.transpose();
    }
    
    Matrix<double, STATE_LENGTH, N> K = P_xz * P_zz.inverse();
    
    State new_state = this_state + K*y;
    CovType new_cov = cov - K*P_zz*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
};


static const Vector3d mag_world = Vector3d(-2341.1e-9, 24138.5e-9, -40313.5e-9); // T
class Nodelet : public nodelet::Nodelet {
  public:
    Nodelet() : last_mag(boost::none), last_good_gps(boost::none), state(boost::none) { }
    
    virtual void onInit() {
      ros::NodeHandle& nh = getNodeHandle();
      imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 10,
        boost::bind(&Nodelet::got_imu, this, _1));
      mag_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/mag", 10,
        boost::bind(&Nodelet::got_mag, this, _1));
      //press_sub = nh.subscribe<sensor_msgs::FluidPressure>("imu/pressure", 10,
      //  boost::bind(&Nodelet::got_press, this, _1));
      gps_sub = nh.subscribe<rawgps_common::Measurements>("gps", 10,
        boost::bind(&Nodelet::got_gps, this, _1));
      
      odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    }
  
  private:
    static const double air_density = 1.225; // kg/m^3
    
    void got_imu(const sensor_msgs::ImuConstPtr &msgp) {
      //const sensor_msgs::Imu &msg = *msgp;
      sensor_msgs::Imu msg = *msgp;
      Map<Matrix3d>(msg.angular_velocity_covariance.data()) =
        pow(0.002, 2)*Matrix3d::Identity();
      Map<Matrix3d>(msg.linear_acceleration_covariance.data()) =
        pow(0.04, 2)*Matrix3d::Identity();
      
      if(state && (msg.header.stamp < state->t || msg.header.stamp > state->t + ros::Duration(.1))) {
        NODELET_ERROR("reset due to invalid stamp");
        last_mag = boost::none;
        state = boost::none;
      }
      if(state && (last_good_gps < ros::Time::now() - ros::Duration(5.))) {
        NODELET_ERROR("reset due to no good gps data");
      }
      
      if(!state) {
        if(last_mag && last_good_gps && *last_good_gps > ros::Time::now() - ros::Duration(1.)) {
          State::StateType stdev = (State::StateType() <<
            0,0,0, .05,.05,.05, .1,.1,.1, .05,.05,.05, 0.1, 1e3).finished();
          AugmentedState::CovType tmp = stdev.asDiagonal();
          Vector3d accel = xyz2vec(msg.linear_acceleration);
          Quaterniond orient = triad(Vector3d(0, 0, -1), mag_world, -accel, *last_mag);
          state = AugmentedState(
            State(msg.header.stamp, Vector3d::Zero(), orient,
              Vector3d::Zero(), Vector3d::Zero(), 9.80665, 101325),
            tmp*tmp);
        }
        return;
      }
      
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //state = state->update(Vector3d::Zero());
      
      std::cout << "gyro_bias " << state->gyro_bias.transpose() << "  " << sqrt(state->cov(9,9)) << " " << sqrt(state->cov(10, 10)) << " " << sqrt(state->cov(11,11)) << std::endl;
      std::cout << "grav: " << state->local_g << "  " << sqrt(state->cov(12,12)) << std::endl;
      std::cout << "ground air pressure: " << state->ground_air_pressure << "  " << sqrt(state->cov(13,13)) << std::endl;
      std::cout << std::endl;
      
      state = state->predict(msg);
      
      fake_depth();
      
      //std::cout << "cov " << state->cov << std::endl << std::endl;
      
      {
        nav_msgs::Odometry output;
        output.header.stamp = msg.header.stamp;
        output.header.frame_id = "/map";
        output.child_frame_id = msg.header.frame_id;
        
        typedef Matrix<double, 6, 6> Matrix6d;
        
        output.pose.pose.position = vec2xyz<geometry_msgs::Point>(state->pos);
        output.pose.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(state->orient);
        Map<Matrix6d>(output.pose.covariance.data()) = state->cov.block<6, 6>(0, 0);
        
        output.twist.twist.linear = vec2xyz<geometry_msgs::Vector3>(state->orient.conjugate()._transformVector(state->vel));
        output.twist.twist.angular = vec2xyz<geometry_msgs::Vector3>(xyz2vec(msg.angular_velocity) - state->gyro_bias);
        Map<Matrix6d> twist_cov(output.twist.covariance.data());
        twist_cov = Matrix6d::Zero();
        twist_cov.block<3, 3>(0, 0) = state->cov.block<3, 3>(6, 6);
        twist_cov.block<3, 3>(3, 3) = Map<Matrix3d>(msg.angular_velocity_covariance.data());
        
        odom_pub.publish(output);
      }
    }
    
    typedef Matrix<double, 1, 1> Matrix1d;
    typedef Matrix<double, 1, 1> Vector1d;
    
    Vector1d mag_observer(const geometry_msgs::Vector3Stamped &msg, const State &state, Vector3d noise) {
      //Vector3d predicted = state.orient.conjugate()._transformVector(mag_world);
      double predicted_angle = atan2(mag_world(1), mag_world(0));
      Vector3d measured_world = state.orient._transformVector(xyz2vec(msg.vector) + noise); // noise shouldn't be here
      double measured_angle = atan2(measured_world(1), measured_world(0));
      double error_angle = measured_angle - predicted_angle;
      double pi = boost::math::constants::pi<double>();
      while(error_angle < pi) error_angle += 2*pi;
      while(error_angle > pi) error_angle -= 2*pi;
      return scalar_matrix(error_angle);
    }
    void got_mag(const geometry_msgs::Vector3StampedConstPtr &msgp) {
      const geometry_msgs::Vector3Stamped &msg = *msgp;
      
      last_mag = xyz2vec(msg.vector);
      
      if(!state) return;
      
      Matrix3d cov = Matrix3d::Zero(); //Map<const Matrix3d>(msg.magnetic_field_covariance.data());
      if(cov == Matrix3d::Zero()) {
        Vector3d stddev(2e-7, 2e-7, 2e-7);
        stddev *= 100;
        cov = stddev.cwiseProduct(stddev).asDiagonal();
      }
      
      state = state->update<1, 3>(
        boost::bind(&Nodelet::mag_observer, this, msg, _1, _2),
      cov);
    }
    
    
    /* Vector1d press_observer(const sensor_msgs::FluidPressure &msg,
                                        const State &state,
                                        Vector1d noise) {
      double predicted = state.ground_air_pressure +
          air_density*Vector3d(0, 0, -state.local_g).dot(state.pos) +
          noise(0);
      return scalar_matrix(msg.fluid_pressure - predicted);
    }
    void got_press(const sensor_msgs::FluidPressureConstPtr &msgp) {
      const sensor_msgs::FluidPressure &msg = *msgp;
      
      Matrix1d cov = scalar_matrix(msg.variance ? msg.variance : pow(10, 2));
      
      if(!state) return;
      
      state = state->update<1, 1>(
        boost::bind(&Nodelet::press_observer, this, msg, _1, _2),
      cov);
    } */
    
    
    VectorXd gps_observer(const rawgps_common::Measurements &msg,
        const State &state, VectorXd noise) {
      VectorXd res(msg.satellites.size());
      for(unsigned int i = 0; i < msg.satellites.size(); i++) {
        const rawgps_common::Satellite &sat = msg.satellites[i];
        double predicted = state.vel.dot(xyz2vec(sat.direction_enu)) + noise(i) + noise(noise.size()-1);
        res[i] = sat.velocity_plus_drift - predicted;
      }
      return res;
    }
    static bool cmp_cn0(const rawgps_common::Satellite &a, const rawgps_common::Satellite &b) {
      return a.cn0 > b.cn0;
    }
    void got_gps(const rawgps_common::MeasurementsConstPtr &msgp) {
      rawgps_common::Measurements msg = *msgp;
      
      double max_cn0 = -std::numeric_limits<double>::infinity();
      BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
        max_cn0 = std::max(max_cn0, satellite.cn0);
      }
      
      if(msg.satellites.size() >= 4) {
        last_good_gps = ros::Time::now();
      } else {
        return;
      }
      
      if(!state) return;
      
      VectorXd stddev(msg.satellites.size() + 1);
      BOOST_FOREACH(const rawgps_common::Satellite &satellite, msg.satellites) {
        stddev[&satellite - msg.satellites.data()] = .15 / sqrt(pow(10, satellite.cn0/10) / pow(10, max_cn0/10));
      }
      stddev[msg.satellites.size()] = 5000;
      std::cout << "stddev:" << std::endl << stddev << std::endl << std::endl;
      
      state = state->update<Dynamic, Dynamic>(
        boost::bind(&Nodelet::gps_observer, this, msg, _1, _2),
      stddev.cwiseProduct(stddev).asDiagonal());
    }
    
    Vector1d depth_observer(double msg, const State &state, Vector1d noise) {
      return scalar_matrix(msg - (-state.pos(2) + noise(0)));
    }
    void fake_depth() {
      state = state->update<1, 1>(boost::bind(&Nodelet::depth_observer, this, 0., _1, _2), scalar_matrix(pow(0.3, 2)));
    }
    
    
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber press_sub;
    ros::Subscriber gps_sub;
    ros::Publisher odom_pub;
    
    boost::optional<Vector3d> last_mag;
    boost::optional<ros::Time> last_good_gps;
    boost::optional<AugmentedState> state;
};

PLUGINLIB_DECLARE_CLASS(odom_estimator, nodelet, odom_estimator::Nodelet, nodelet::Nodelet);

}
