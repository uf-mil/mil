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
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"
#include "odom_estimator/state.h"
#include "odom_estimator/kalman.h"
#include "odom_estimator/gps.h"

using namespace Eigen;

namespace odom_estimator {




class NodeImpl {
  private:
    boost::function<const std::string&()> getName;
    ros::NodeHandle &nh;
    ros::NodeHandle &private_nh;
    Vector3d mag_world;
    constexpr static const double air_density = 1.225; // kg/m^3
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
          Matrix<double, State::RowsAtCompileTime, 1> stdev =
            (Matrix<double, State::RowsAtCompileTime, 1>() <<
            0,0,0, .05,.05,.05, 1,1,1, 1e-3,1e-3,1e-3, 0.1, 1e3).finished();
          Matrix<double, State::RowsAtCompileTime, State::RowsAtCompileTime> tmp =
            stdev.asDiagonal();
          Vector3d accel = xyz2vec(msg.linear_acceleration);
          Quaterniond orient = triad(Vector3d(0, 0, -1), mag_world, -accel, *last_mag);
          state = AugmentedState<State>(
            State(msg.header.stamp, start_pos, orient,
              Vector3d::Zero(), Vector3d::Zero(), 9.80665, 101325),
            tmp*tmp);
        }
      } else {
        state = state->predict<StateUpdater>(StateUpdater(msg));
      }
      
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //state = state->update(Vector3d::Zero());
      
      std::cout << "gyro_bias " << state->gyro_bias.transpose()
        << " " << sqrt(state->cov(State::GYRO_BIAS+0, State::GYRO_BIAS+0))
        << " " << sqrt(state->cov(State::GYRO_BIAS+1, State::GYRO_BIAS+1))
        << " " << sqrt(state->cov(State::GYRO_BIAS+2, State::GYRO_BIAS+2)) << std::endl;
      std::cout << "grav: " << state->local_g << "  " << sqrt(state->cov(State::LOCAL_G, State::LOCAL_G)) << std::endl;
      std::cout << "ground air pressure: " << state->ground_air_pressure << "  " << sqrt(state->cov(State::GROUND_AIR_PRESSURE, State::GROUND_AIR_PRESSURE)) << std::endl;
      std::cout << std::endl;
      

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
        
        tf::pointEigenToMsg(state->pos, output.pose.pose.position);
        tf::quaternionEigenToMsg(state->orient, output.pose.pose.orientation);
        Map<Matrix6d>(output.pose.covariance.data()) <<
          state->cov.block<3, 3>(State::POS, State::POS), state->cov.block<3, 3>(State::POS, State::ORIENT),
          state->cov.block<3, 3>(State::ORIENT, State::POS), state->cov.block<3, 3>(State::ORIENT, State::ORIENT);
        
        tf::vectorEigenToMsg(state->orient.conjugate()._transformVector(state->vel), output.twist.twist.linear);
        tf::vectorEigenToMsg(xyz2vec(msg.angular_velocity) - state->gyro_bias, output.twist.twist.angular);
        Map<Matrix6d>(output.twist.covariance.data()) <<
          state->cov.block<3, 3>(State::VEL, State::VEL), Matrix3d::Zero(), // XXX covariance needs to be adjusted since velocity was transformed to body frame
          Matrix3d::Zero(), Map<Matrix3d>(msg.angular_velocity_covariance.data()) + state->cov.block<3, 3>(State::GYRO_BIAS, State::GYRO_BIAS);
        
        odom_pub.publish(output);
      }
    }
    
    typedef Matrix<double, 1, 1> Matrix1d;
    typedef Matrix<double, 1, 1> Vector1d;
    
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
      
      state = state->update<1, 3>([&msg, this](const StateWithMeasurementNoise<3> &tmp) {
        State const &state = tmp.first;
        Matrix<double, 3, 1> measurement_noise = tmp.second;
        //Vector3d predicted = state.orient.conjugate()._transformVector(mag_world);
        double predicted_angle = atan2(mag_world(1), mag_world(0));
        Vector3d measured_world = state.orient._transformVector(xyz2vec(msg.magnetic_field) + measurement_noise); // noise shouldn't be here
        double measured_angle = atan2(measured_world(1), measured_world(0));
        double error_angle = measured_angle - predicted_angle;
        double pi = boost::math::constants::pi<double>();
        while(error_angle < pi) error_angle += 2*pi;
        while(error_angle > pi) error_angle -= 2*pi;
        return scalar_matrix(error_angle);
      }, cov);
    }
    
    
    Vector1d press_observer(const sensor_msgs::FluidPressure &msg,
                               const StateWithMeasurementNoise<1> &tmp) {
      State const &state = tmp.first;
      Matrix<double, 1, 1> measurement_noise = tmp.second;
      double predicted = state.ground_air_pressure +
          air_density*Vector3d(0, 0, -state.local_g).dot(state.pos) +
          measurement_noise(0);
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
    
    void got_gps(const rawgps_common::MeasurementsConstPtr &msgp) {
      rawgps_common::Measurements msg = *msgp;
      
      tf::StampedTransform transform;
      try {
        tf_listener.lookupTransform(local_frame_id,
          msg.header.frame_id, msg.header.stamp, transform);
      } catch (tf::TransformException ex) {
        NODELET_ERROR("Error in got_gps: %s", ex.what());
        return;
      }
      Vector3d local_gps_pos; tf::vectorTFToEigen(transform.getOrigin(), local_gps_pos);
      
      if(msg.satellites.size() >= 4) {
        last_good_gps = ros::Time::now();
      } else {
        std::cout << "bad gps" << std::endl;
      }
      
      if(!state) return;
      
      state = state->update<GPSErrorObserver>(
        GPSErrorObserver(msg, local_gps_pos, *last_gyro));
    }
    
    
    Vector1d depth_observer(double msg, const StateWithMeasurementNoise<1> &tmp) {
      State const &state = tmp.first;
      Matrix<double, 1, 1> measurement_noise = tmp.second;
      return scalar_matrix(msg - (-state.pos(2) + measurement_noise(0)));
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
    boost::optional<AugmentedState<State> > state;
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
