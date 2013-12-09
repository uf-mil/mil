#include <boost/bind.hpp>
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
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <rawgps_common/Measurements.h>

#include <odom_estimator/Info.h>

#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"
#include "odom_estimator/state.h"
#include "odom_estimator/earth.h"
#include "odom_estimator/magnetic.h"
#include "odom_estimator/kalman.h"
#include "odom_estimator/gps.h"

namespace odom_estimator {


GaussianDistribution<State> init_state(sensor_msgs::Imu const &msg,
                                       Vec<3> last_mag,
                                       Vec<3> pos_ecef,
                                       Vec<3> vel_ecef) {
  Vec<3> pos_eci = inertial_from_ecef(
    msg.header.stamp.toSec(), pos_ecef);
  Vec<3> vel_eci = inertial_vel_from_ecef_vel(
    msg.header.stamp.toSec(), vel_ecef, pos_eci);
  
  Vec<3> mag_eci = magnetic::getMagneticField(pos_eci);
  Vec<3> predicted_acc_eci = inertial_acc_from_ecef_acc(
    msg.header.stamp.toSec(), Vec<3>::Zero(), pos_eci);
  Vec<3> predicted_accelerometer_eci =
    predicted_acc_eci - gravity::gravity(pos_eci);
  Vec<3> accel_body = xyz2vec(msg.linear_acceleration);
  Quaternion orient_eci = triad(
    predicted_accelerometer_eci, mag_eci,
    accel_body, last_mag);
  
  Vec<State::RowsAtCompileTime> stdev =
    (Vec<State::RowsAtCompileTime>(14) <<
    0,0,0, .05,.05,.05, 1,1,1, 1e-3,1e-3,1e-3, 0.1, 1e3).finished();
  SqMat<State::RowsAtCompileTime> tmp =
    stdev.asDiagonal();
  
  return GaussianDistribution<State>(
    State(msg.header.stamp,
      pos_eci,
      orient_eci,
      vel_eci,
      Vec<3>::Zero(),
      9.80665,
      101325,
      std::vector<int>{},
      Vec<Dynamic>::Zero(0)),
    tmp*tmp);
}

GaussianDistribution<State>
init_state(sensor_msgs::Imu const &msg,
           Vec<3> last_mag,
           rawgps_common::Measurements const &gps) {
  GaussianDistribution<Fix> fix = get_fix(gps);
  return init_state(msg, last_mag, fix.mean.first, fix.mean.second);
}


class NodeImpl {
  private:
    boost::function<const std::string&()> getName;
    ros::NodeHandle &nh;
    ros::NodeHandle &private_nh;
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
      info_pub = private_nh.advertise<odom_estimator::Info>("info", 10);
    }
  
  private:
    
    void got_imu(const sensor_msgs::ImuConstPtr &msgp) {
      //const sensor_msgs::Imu &msg = *msgp;
      sensor_msgs::Imu msg = *msgp;
      Eigen::Map<SqMat<3> >(msg.angular_velocity_covariance.data()) =
        pow(0.02, 2)*SqMat<3>::Identity();
      Eigen::Map<SqMat<3> >(msg.linear_acceleration_covariance.data()) =
        pow(0.06, 2)*SqMat<3>::Identity();
      
      gps_filter.setTargetFrame(msg.header.frame_id);
      last_gyro = xyz2vec(msg.angular_velocity);
      local_frame_id = msg.header.frame_id;
      
      if(state && (msg.header.stamp < state->mean.t || msg.header.stamp > state->mean.t + ros::Duration(2))) {
        NODELET_ERROR("reset due to invalid stamp");
        last_mag = boost::none;
        start_pos = state->mean.getPosECEF();
        state = boost::none;
      }
      if(state && (last_good_gps < msg.header.stamp - ros::Duration(5.))) {
        NODELET_ERROR("reset due to no good gps data");
      }
      
      if(!state) {
        if(last_mag && last_good_gps && *last_good_gps > msg.header.stamp - ros::Duration(1.5) &&
                                        *last_good_gps < msg.header.stamp + ros::Duration(1.5)) {
          state = init_state(msg, *last_mag, *last_good_gps_msg);
        } else {
          std::cout << "something missing" << std::endl;
          return;
        }
      } else {
        state = StateUpdater(msg)(*state);
      }
      
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //state = state->update(Vec<3>::Zero());
      
      std::cout << "gyro_bias " << state->mean.gyro_bias.transpose()
        << " " << sqrt(state->cov(State::GYRO_BIAS+0, State::GYRO_BIAS+0))
        << " " << sqrt(state->cov(State::GYRO_BIAS+1, State::GYRO_BIAS+1))
        << " " << sqrt(state->cov(State::GYRO_BIAS+2, State::GYRO_BIAS+2)) << std::endl;
      std::cout << "grav: " << state->mean.local_g << "  " << sqrt(state->cov(State::LOCAL_G, State::LOCAL_G)) << std::endl;
      std::cout << "ground air pressure: " << state->mean.ground_air_pressure << "  " << sqrt(state->cov(State::GROUND_AIR_PRESSURE, State::GROUND_AIR_PRESSURE)) << std::endl;
      for(unsigned int i = 0; i < state->mean.gps_prn.size(); i++) {
        std::cout << state->mean.gps_prn[i]
          << " " << state->mean.gps_bias[i]
          << " " << sqrt(state->cov(State::GPS_BIAS + i, State::GPS_BIAS + i))
          << std::endl;
      }
      std::cout << std::endl;
      

      if(state->mean.gyro_bias.norm() > .5) {
        NODELET_ERROR("reset due to bad gyro biases");
        last_mag = boost::none;
        start_pos = state->mean.getPosECEF();
        state = boost::none;
        return;
      }

      //std::cout << "cov " << state->cov << std::endl << std::endl;
      
      {
        nav_msgs::Odometry output;
        output.header.stamp = msg.header.stamp;
        output.header.frame_id = "/ecef";
        output.child_frame_id = msg.header.frame_id;
        
        tf::pointEigenToMsg(state->mean.getPosECEF(), output.pose.pose.position);
        tf::quaternionEigenToMsg(state->mean.getOrientECEF(), output.pose.pose.orientation);
        Eigen::Map<SqMat<6> >(output.pose.covariance.data()) <<
          state->cov.block<3, 3>(State::POS_ECI, State::POS_ECI), state->cov.block<3, 3>(State::POS_ECI, State::ORIENT),
          state->cov.block<3, 3>(State::ORIENT, State::POS_ECI), state->cov.block<3, 3>(State::ORIENT, State::ORIENT);
        
        tf::vectorEigenToMsg(state->mean.getOrientECEF().conjugate()._transformVector(state->mean.getVelECEF()), output.twist.twist.linear);
        tf::vectorEigenToMsg(xyz2vec(msg.angular_velocity) - state->mean.gyro_bias, output.twist.twist.angular);
        Eigen::Map<SqMat<6> >(output.twist.covariance.data()) <<
          state->cov.block<3, 3>(State::VEL, State::VEL), SqMat<3>::Zero(), // XXX covariance needs to be adjusted since velocity was transformed to body frame
          SqMat<3>::Zero(), Eigen::Map<SqMat<3> >(msg.angular_velocity_covariance.data()) + state->cov.block<3, 3>(State::GYRO_BIAS, State::GYRO_BIAS);
        
        odom_pub.publish(output);
      }
      
      {
        odom_estimator::Info output;
        output.header.stamp = msg.header.stamp;
        
        for(unsigned int i = 0; i < state->mean.gps_prn.size(); i++) {
          output.gps_bias_prns.push_back(state->mean.gps_prn[i]);
          output.gps_bias_biases.push_back(state->mean.gps_bias[i]);
          output.gps_bias_stddevs.push_back(sqrt(state->cov(State::GPS_BIAS + i, State::GPS_BIAS + i)));
        }
        
        info_pub.publish(output);
      }
    }
    
    void got_mag(const sensor_msgs::MagneticFieldConstPtr &msgp) {
      const sensor_msgs::MagneticField &msg = *msgp;
      
      last_mag = xyz2vec(msg.magnetic_field);
      
      if(!state) return;
      
      SqMat<3> cov = Eigen::Map<const SqMat<3> >(msg.magnetic_field_covariance.data());
      if(cov == SqMat<3>::Zero()) {
        Vec<3> stddev(2e-7, 2e-7, 2e-7);
        stddev *= 100;
        cov = stddev.cwiseProduct(stddev).asDiagonal();
      }
      
      state = kalman_update(
        EasyDistributionFunction<State, Vec<1>, Vec<3> >(
          [&msg, this](State const &state, Vec<3> const &measurement_noise) {
            Vec<3> mag_world = magnetic::getMagneticField(state.pos_eci);
            //Vec<3> predicted = state.orient.conjugate()._transformVector(mag_world);
            double predicted_angle = atan2(mag_world(1), mag_world(0));
            Vec<3> measured_world = state.orient._transformVector(xyz2vec(msg.magnetic_field) + measurement_noise); // noise shouldn't be here
            double measured_angle = atan2(measured_world(1), measured_world(0));
            double error_angle = measured_angle - predicted_angle;
            double pi = boost::math::constants::pi<double>();
            while(error_angle < pi) error_angle += 2*pi;
            while(error_angle > pi) error_angle -= 2*pi;
            return scalar_matrix(error_angle);
          },
          GaussianDistribution<Vec<3> >(Vec<3>::Zero(), cov)),
        *state);
    }
    
    
    void got_press(const sensor_msgs::FluidPressureConstPtr &msgp) {
      /*const sensor_msgs::FluidPressure &msg = *msgp;
      
      if(!state) return;
      
      state = kalman_update(
        EasyDistributionFunction<State, Vec<1>, Vec<1> >(
          [&msg](State const &state, Vec<1> const &measurement_noise) {
            double predicted = state.ground_air_pressure +
              air_density*Vec<3>(0, 0, -state.local_g).dot(state.pos) +
              measurement_noise(0);
            return scalar_matrix(msg.fluid_pressure - predicted);
          },
          GaussianDistribution<Vec<1> >(
            Vec<1>::Zero(),
            scalar_matrix(msg.variance ? msg.variance : pow(10, 2)))),
        *state);*/
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
      Vec<3> local_gps_pos; tf::vectorTFToEigen(transform.getOrigin(), local_gps_pos);
      
      if(msg.satellites.size() >= 4) {
        last_good_gps = msg.header.stamp;
        last_good_gps_msg = msg;
        std::cout << "got gps" << std::endl;
      } else {
        std::cout << "bad gps" << std::endl;
      }
      
      if(!state) return;
      
      std::vector<int> new_prns;
      std::vector<rawgps_common::Satellite> const &sats = msg.satellites;
      for(unsigned int i = 0; i < sats.size(); i++) {
        rawgps_common::Satellite const &sat = sats[i];
        if(std::find(state->mean.gps_prn.begin(), state->mean.gps_prn.end(), sat.prn) == state->mean.gps_prn.end()) {
          new_prns.push_back(sat.prn);
        }
      }
      
      state = EasyDistributionFunction<State, State, Vec<Dynamic> >(
        [&new_prns](State const &state, Vec<Dynamic> const &new_bias) {
          State new_state = state;
          for(unsigned int i = 0; i < new_prns.size(); i++) {
            new_state.gps_prn.push_back(new_prns[i]);
          }
          new_state.gps_bias = 
            (Vec<Dynamic>(state.gps_bias.rows()+new_bias.rows())
              << state.gps_bias, new_bias).finished();
          return new_state;
        },
        GaussianDistribution<Vec<Dynamic> >(
          Vec<Dynamic>::Zero(new_prns.size()),
          pow(10, 2)*Vec<Dynamic>::Ones(new_prns.size()).asDiagonal())
      )(*state);
      
      state = kalman_update(
        GPSErrorObserver(msg, state->mean.gps_prn, local_gps_pos, *last_gyro),
        *state);
    }
    
    
    /*void fake_depth() {
      state = kalman_update(
        EasyDistributionFunction<State, Vec<1>, Vec<1> >(
          [](State const &state, Vec<1> const &measurement_noise) {
            double measured = 0;
            double predicted = -state.pos(2) + measurement_noise(0);
            return scalar_matrix(measured - predicted);
          },
          GaussianDistribution<Vec<1> >(
            Vec<1>::Zero(),
            scalar_matrix(pow(0.3, 2)))),
        *state);
    }*/
    
    
    tf::TransformListener tf_listener;
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber press_sub;
    message_filters::Subscriber<rawgps_common::Measurements> gps_sub;
    tf::MessageFilter<rawgps_common::Measurements> gps_filter;
    ros::Publisher odom_pub;
    ros::Publisher info_pub;
    
    Vec<3> start_pos;
    boost::optional<Vec<3> > last_mag;
    boost::optional<ros::Time> last_good_gps;
    boost::optional<rawgps_common::Measurements> last_good_gps_msg;
    boost::optional<GaussianDistribution<State> > state;
    boost::optional<Vec<3> > last_gyro;
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
