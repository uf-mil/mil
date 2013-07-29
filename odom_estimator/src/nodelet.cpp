#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <rawgps_common/Measurements.h>

#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"

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
  static const int DELTA_SIZE = 3*4 + 2;
  typedef Matrix<double, DELTA_SIZE, 1> DeltaType;
  
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
      pos + other.segment<3>(0),
      quat_from_rotvec(other.segment<3>(3)) * orient,
      vel + other.segment<3>(6),
      gyro_bias + other.segment<3>(9),
      local_g + other(12),
      ground_air_pressure + other(13));
  }
};

class StateWithImuDataAndProcessNoise : public State {
private:
  Vector3d angular_velocity, linear_acceleration;
  Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> process_noise;
public:
  static const unsigned int SIZE = State::DELTA_SIZE + 6 + PREDICT_EXTRA_NOISE_LENGTH;
  StateWithImuDataAndProcessNoise(const State &state,
      Vector3d angular_velocity, Vector3d linear_acceleration,
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> noise) :
    State(state), angular_velocity(angular_velocity),
    linear_acceleration(linear_acceleration), process_noise(process_noise) { }
  Matrix<double, SIZE, 1> operator-(const StateWithImuDataAndProcessNoise &other) const {
    return (Matrix<double, SIZE, 1>() <<
      static_cast<const State&>(*this) - static_cast<const State&>(other),
      angular_velocity - other.angular_velocity,
      linear_acceleration - other.linear_acceleration,
      process_noise - other.process_noise).finished();
  }
  StateWithImuDataAndProcessNoise operator+(const Matrix<double, SIZE, 1> &other) const {
    return StateWithImuDataAndProcessNoise(
      static_cast<const State&>(*this) + other.segment<State::DELTA_SIZE>(0),
      angular_velocity + other.segment<3>(State::DELTA_SIZE),
      linear_acceleration + other.segment<3>(State::DELTA_SIZE + 3),
      process_noise + other.segment<PREDICT_EXTRA_NOISE_LENGTH>(State::DELTA_SIZE + 6));
  }
  State predict(ros::Time t) const {
    return static_cast<const State&>(*this).predict(t,
      angular_velocity, linear_acceleration, process_noise);
  }
};

template<int NN>
class StateWithMeasurementNoise : public State {
public:
  Matrix<double, NN, 1> measurement_noise;
  static const unsigned int DELTA_SIZE = NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic;
  typedef Matrix<double, DELTA_SIZE, 1> DeltaType;
  StateWithMeasurementNoise(const State &state, Matrix<double, NN, 1> measurement_noise) :
    State(state), measurement_noise(measurement_noise) { }
  DeltaType operator-(const StateWithMeasurementNoise<NN> &other) {
    return (DeltaType() <<
        static_cast<const State&>(*this) - static_cast<const State&>(other),
        measurement_noise - other.measurement_noise).finished();
  }
  StateWithMeasurementNoise<NN> operator+(DeltaType &other) const {
    return StateWithMeasurementNoise<NN>(
      static_cast<const State&>(*this) + other.template segment<State::DELTA_SIZE>(0),
      measurement_noise + other.tail(other.rows() - State::DELTA_SIZE));
  }
};

struct AugmentedState : public State {
  typedef Matrix<double, State::DELTA_SIZE, State::DELTA_SIZE> CovType;
  CovType cov;
  AugmentedState(const State &state, const CovType &cov) :
    State(state), cov(cov/2 + cov.transpose()/2) { }
  
  AugmentedState predict(const sensor_msgs::Imu &imu) const {
    const unsigned int NOISE_LENGTH = 3*2+PREDICT_EXTRA_NOISE_LENGTH;
    
    StateWithImuDataAndProcessNoise mean = StateWithImuDataAndProcessNoise(
      static_cast<const State&>(*this),
      xyz2vec(imu.angular_velocity),
      xyz2vec(imu.linear_acceleration),
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1>::Zero());
    
    typedef Matrix<double, State::DELTA_SIZE + NOISE_LENGTH,
      State::DELTA_SIZE + NOISE_LENGTH> AugmentedMatrixType;
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero();
    Pa.block<State::DELTA_SIZE, State::DELTA_SIZE>(0, 0) = cov;
    Pa.block<3, 3>(State::DELTA_SIZE, State::DELTA_SIZE) =
      Map<const Matrix3d>(imu.angular_velocity_covariance.data());
    Pa.block<3, 3>(State::DELTA_SIZE + 3, State::DELTA_SIZE + 3) =
      Map<const Matrix3d>(imu.linear_acceleration_covariance.data());
    Pa.block<PREDICT_EXTRA_NOISE_LENGTH, PREDICT_EXTRA_NOISE_LENGTH>(
      State::DELTA_SIZE + 6, State::DELTA_SIZE + 6) = get_extra_noise_cov();
    
    UnscentedTransform<State, State::DELTA_SIZE,
      StateWithImuDataAndProcessNoise, State::DELTA_SIZE + NOISE_LENGTH> res(
        boost::bind(&StateWithImuDataAndProcessNoise::predict, _1, imu.header.stamp),
        mean, Pa);
    
    return AugmentedState(res.mean, res.cov);
  }
  
  template <int N, int NN>
  AugmentedState update(
      const boost::function<Matrix<double, N, 1> (StateWithMeasurementNoise<NN>)> &observe,
      const Matrix<double, NN, NN> &noise_cov) const {
    unsigned int realNN = NN != Dynamic ? NN : noise_cov.rows();
    assert(noise_cov.rows() == noise_cov.cols());
    
    StateWithMeasurementNoise<NN> mean = StateWithMeasurementNoise<NN>(
      static_cast<const State&>(*this), Matrix<double, NN, 1>::Zero(realNN));
    typedef Matrix<double, NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic,
                           NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic>
      AugmentedMatrixType;
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero(
      State::DELTA_SIZE + realNN, State::DELTA_SIZE + realNN);
    Pa.template block<State::DELTA_SIZE, State::DELTA_SIZE>(0, 0) = cov;
    Pa.template bottomRightCorner(realNN, realNN) = noise_cov;
    
    UnscentedTransform<Matrix<double, N, 1>, N,
      StateWithMeasurementNoise<NN>, NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic>
      res(observe, mean, Pa);
    unsigned int realN = res.mean.rows();
    
    Matrix<double, State::DELTA_SIZE, N> K =
      res.cross_cov.transpose().template topLeftCorner(State::DELTA_SIZE, realN) *
      res.cov.inverse();
    
    State new_state = static_cast<const State&>(*this) + K*-res.mean;
    CovType new_cov = cov - K*res.cov*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
};


class NodeImpl {
  private:
    boost::function<const std::string&()> getName;
    ros::NodeHandle &nh;
    ros::NodeHandle &private_nh;
    Vector3d mag_world;
    static const double air_density = 1.225; // kg/m^3
  public:
    NodeImpl(boost::function<const std::string&()> getName, ros::NodeHandle *nh_, ros::NodeHandle *private_nh_) :
        getName(getName),
        nh(*nh_),
        private_nh(*private_nh_),
        gps_sub(nh, "gps", 1),
        gps_filter(gps_sub, tf_listener, "", 10),
        start_pos(0, 0, 0),
        last_mag(boost::none), last_good_gps(boost::none), state(boost::none) {
      imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 10,
        boost::bind(&NodeImpl::got_imu, this, _1));
      mag_sub = nh.subscribe<sensor_msgs::MagneticField>("imu/mag", 10,
        boost::bind(&NodeImpl::got_mag, this, _1));
      press_sub = nh.subscribe<sensor_msgs::FluidPressure>("imu/pressure", 10,
        boost::bind(&NodeImpl::got_press, this, _1));
      gps_filter.registerCallback(boost::bind(&NodeImpl::got_gps, this, _1));
      odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
      
      mag_world = Vector3d(-2341.1e-9, 24138.5e-9, -40313.5e-9); // T
    }
  
  private:
    
    void got_imu(const sensor_msgs::ImuConstPtr &msgp) {
      //const sensor_msgs::Imu &msg = *msgp;
      sensor_msgs::Imu msg = *msgp;
      Map<Matrix3d>(msg.angular_velocity_covariance.data()) =
        pow(0.02, 2)*Matrix3d::Identity();
      Map<Matrix3d>(msg.linear_acceleration_covariance.data()) =
        pow(0.06, 2)*Matrix3d::Identity();
      
      gps_filter.setTargetFrame(msg.header.frame_id);
      last_gyro = xyz2vec(msg.angular_velocity);
      local_frame_id = msg.header.frame_id;
      
      if(state && (msg.header.stamp < state->t || msg.header.stamp > state->t + ros::Duration(2))) {
        NODELET_ERROR("reset due to invalid stamp");
        last_mag = boost::none;
        start_pos = state->pos;
        state = boost::none;
      }
      if(state && (last_good_gps < ros::Time::now() - ros::Duration(5.))) {
        NODELET_ERROR("reset due to no good gps data");
      }
      
      if(!state) {
        if(last_mag && last_good_gps && *last_good_gps > ros::Time::now() - ros::Duration(1.)) {
          State::DeltaType stdev = (State::DeltaType() <<
            0,0,0, .05,.05,.05, .1,.1,.1, .05,.05,.05, 0.1, 1e3).finished();
          AugmentedState::CovType tmp = stdev.asDiagonal();
          Vector3d accel = xyz2vec(msg.linear_acceleration);
          Quaterniond orient = triad(Vector3d(0, 0, -1), mag_world, -accel, *last_mag);
          state = AugmentedState(
            State(msg.header.stamp, start_pos, orient,
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
      

      if(state->gyro_bias.norm() > .5) {
        NODELET_ERROR("reset due to bad gyro biases");
        last_mag = boost::none;
        start_pos = state->pos;
        state = boost::none;
        return;
      }

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
        twist_cov.block<3, 3>(3, 3) = Map<Matrix3d>(msg.angular_velocity_covariance.data()) + state->cov.block<3, 3>(9, 9);
        
        odom_pub.publish(output);
      }
    }
    
    typedef Matrix<double, 1, 1> Matrix1d;
    typedef Matrix<double, 1, 1> Vector1d;
    
    Vector1d mag_observer(const sensor_msgs::MagneticField &msg, const StateWithMeasurementNoise<3> &state) {
      //Vector3d predicted = state.orient.conjugate()._transformVector(mag_world);
      double predicted_angle = atan2(mag_world(1), mag_world(0));
      Vector3d measured_world = state.orient._transformVector(xyz2vec(msg.magnetic_field) + state.measurement_noise); // noise shouldn't be here
      double measured_angle = atan2(measured_world(1), measured_world(0));
      double error_angle = measured_angle - predicted_angle;
      double pi = boost::math::constants::pi<double>();
      while(error_angle < pi) error_angle += 2*pi;
      while(error_angle > pi) error_angle -= 2*pi;
      return scalar_matrix(error_angle);
    }
    void got_mag(const sensor_msgs::MagneticFieldConstPtr &msgp) {
      const sensor_msgs::MagneticField &msg = *msgp;
      
      last_mag = xyz2vec(msg.magnetic_field);
      
      if(!state) return;
      
      Matrix3d cov = Map<const Matrix3d>(msg.magnetic_field_covariance.data());
      if(cov == Matrix3d::Zero()) {
        Vector3d stddev(2e-7, 2e-7, 2e-7);
        stddev *= 100;
        cov = stddev.cwiseProduct(stddev).asDiagonal();
      }
      
      state = state->update<1, 3>(
        boost::bind(&NodeImpl::mag_observer, this, msg, _1),
      cov);
    }
    
    
    Vector1d press_observer(const sensor_msgs::FluidPressure &msg,
                               const StateWithMeasurementNoise<1> &state) {
      double predicted = state.ground_air_pressure +
          air_density*Vector3d(0, 0, -state.local_g).dot(state.pos) +
          state.measurement_noise(0);
      return scalar_matrix(msg.fluid_pressure - predicted);
    }
    void got_press(const sensor_msgs::FluidPressureConstPtr &msgp) {
      const sensor_msgs::FluidPressure &msg = *msgp;
      
      Matrix1d cov = scalar_matrix(msg.variance ? msg.variance : pow(10, 2));
      
      if(!state) return;
      
      state = state->update<1, 1>(
        boost::bind(&NodeImpl::press_observer, this, msg, _1),
      cov);
    }
    
    
    VectorXd gps_observer(const rawgps_common::Measurements &msg,
        const Vector3d &gps_pos, const StateWithMeasurementNoise<Dynamic> &state) {
      VectorXd res(msg.satellites.size());
      Vector3d gps_vel = state.vel + state.orient._transformVector(
        (*last_gyro - state.gyro_bias).cross(gps_pos));
      for(unsigned int i = 0; i < msg.satellites.size(); i++) {
        const rawgps_common::Satellite &sat = msg.satellites[i];
        double predicted = gps_vel.dot(xyz2vec(sat.direction_enu)) +
          state.measurement_noise(i) + state.measurement_noise(state.measurement_noise.size()-1);
        res[i] = sat.velocity_plus_drift - predicted;
      }
      return res;
    }
    static bool cmp_cn0(const rawgps_common::Satellite &a, const rawgps_common::Satellite &b) {
      return a.cn0 > b.cn0;
    }
    void got_gps(const rawgps_common::MeasurementsConstPtr &msgp) {
      rawgps_common::Measurements msg = *msgp;
      
      tf::StampedTransform transform;
      try {
        tf_listener.lookupTransform(local_frame_id,
          msg.header.frame_id, msg.header.stamp, transform);
      } catch (tf::TransformException ex) {
        NODELET_ERROR("%s", ex.what());
        return;
      }
      Vector3d local_gps_pos(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      
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
        boost::bind(&NodeImpl::gps_observer, this, msg, local_gps_pos, _1),
      stddev.cwiseProduct(stddev).asDiagonal());
    }
    
    Vector1d depth_observer(double msg, const StateWithMeasurementNoise<1> &state) {
      return scalar_matrix(msg - (-state.pos(2) + state.measurement_noise(0)));
    }
    void fake_depth() {
      state = state->update<1, 1>(boost::bind(&NodeImpl::depth_observer, this, 0., _1), scalar_matrix(pow(0.3, 2)));
    }
    
    
    tf::TransformListener tf_listener;
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber press_sub;
    message_filters::Subscriber<rawgps_common::Measurements> gps_sub;
    tf::MessageFilter<rawgps_common::Measurements> gps_filter;
    ros::Publisher odom_pub;
    
    Vector3d start_pos;
    boost::optional<Vector3d> last_mag;
    boost::optional<ros::Time> last_good_gps;
    boost::optional<AugmentedState> state;
    boost::optional<Vector3d> last_gyro;
    std::string local_frame_id;
};

class Nodelet : public nodelet::Nodelet {
public:
    Nodelet() { }
    
    virtual void onInit() {
        nodeimpl = boost::in_place(boost::bind(&Nodelet::getName, this),
          &getNodeHandle(), &getPrivateNodeHandle());
    }

private:
    boost::optional<NodeImpl> nodeimpl;
};
PLUGINLIB_DECLARE_CLASS(odom_estimator, nodelet, odom_estimator::Nodelet, nodelet::Nodelet);

}
