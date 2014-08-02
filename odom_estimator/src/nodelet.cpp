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
#include <uf_common/VelocityMeasurements.h>
#include <uf_common/Float64Stamped.h>

#include <odom_estimator/Info.h>
#include <odom_estimator/SetIgnoreMagnetometer.h>

#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"
#include "odom_estimator/state.h"
#include "odom_estimator/earth.h"
#include "odom_estimator/magnetic.h"
#include "odom_estimator/kalman.h"
#include "odom_estimator/gps.h"
#include "odom_estimator/odometry.h"

namespace odom_estimator {


static magnetic::MagneticModel const magnetic_model(ros::package::getPath("odom_estimator") + "/data/WMM.COF");

GaussianDistribution<State> init_state(sensor_msgs::Imu const &msg,
                                       Vec<3> last_mag,
                                       Vec<3> pos_ecef,
                                       Vec<3> vel_ecef) {
  Vec<3> pos_eci = inertial_from_ecef(
    msg.header.stamp.toSec(), pos_ecef);
  Vec<3> vel_eci = inertial_vel_from_ecef_vel(
    msg.header.stamp.toSec(), vel_ecef, pos_eci);
  
  Vec<3> mag_eci = magnetic_model.getField(pos_eci, msg.header.stamp.toSec());
  Vec<3> predicted_acc_eci = inertial_acc_from_ecef_acc(
    msg.header.stamp.toSec(), Vec<3>::Zero(), pos_eci);
  Vec<3> predicted_accelerometer_eci =
    predicted_acc_eci - gravity::gravity(pos_eci);
  Vec<3> accel_body = xyz2vec(msg.linear_acceleration);
  Quaternion orient_eci = triad(
    predicted_accelerometer_eci, mag_eci,
    accel_body, last_mag);
  
  Vec<State::RowsAtCompileTime> stdev =
    (Vec<State::RowsAtCompileTime>(20) <<
    100,100,100, 0,0,0, .05,.05,.05, 10,10,10, 1e-3,1e-3,1e-3, 1e-2,1e-2,1e-2, 0.1, 1e3).finished();
  SqMat<State::RowsAtCompileTime> tmp =
    stdev.asDiagonal();
  
  return GaussianDistribution<State>(
    State(msg.header.stamp,
      msg.header.stamp,
      std::vector<int>{},
      pos_eci,
      Vec<3>::Zero(),
      orient_eci,
      vel_eci,
      Vec<3>::Zero(),
      Vec<3>::Zero(),
      9.80665,
      101325,
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
    bool have_gps;
    double start_x_ecef, start_y_ecef, start_z_ecef;
    constexpr static const double air_density = 1.225; // kg/m^3
    std::string local_frame;
    ros::ServiceServer set_ignore_magnetometer_srv;
    bool ignoreMagnetometer;
  public:
    NodeImpl(boost::function<const std::string&()> getName, ros::NodeHandle *nh_, ros::NodeHandle *private_nh_) :
        getName(getName),
        nh(*nh_),
        private_nh(*private_nh_),
        have_gps(true),
        local_frame("/enu"),
        ignoreMagnetometer(false),
        gps_sub(nh, "gps", 1),
        gps_filter(gps_sub, tf_listener, "", 10),
        dvl_sub(nh, "dvl", 1),
        dvl_filter(dvl_sub, tf_listener, "", 10),
        depth_sub(nh, "depth", 1),
        depth_filter(depth_sub, tf_listener, "", 10),
        last_mag(boost::none), last_good_gps(boost::none), state(boost::none) {
      
      private_nh.getParam("have_gps", have_gps);
      private_nh.getParam("start_x_ecef", start_x_ecef);
      private_nh.getParam("start_y_ecef", start_y_ecef);
      private_nh.getParam("start_z_ecef", start_z_ecef);
      private_nh.getParam("local_frame", local_frame);
      
      imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 10,
        boost::bind(&NodeImpl::got_imu, this, _1));
      mag_sub = nh.subscribe<sensor_msgs::MagneticField>("imu/mag", 10,
        boost::bind(&NodeImpl::got_mag, this, _1));
      press_sub = nh.subscribe<sensor_msgs::FluidPressure>("imu/pressure", 10,
        boost::bind(&NodeImpl::got_press, this, _1));
      gps_filter.registerCallback(boost::bind(&NodeImpl::got_gps, this, _1));
      dvl_filter.registerCallback(boost::bind(&NodeImpl::got_dvl, this, _1));
      depth_filter.registerCallback(boost::bind(&NodeImpl::got_depth, this, _1));
      odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
      absodom_pub = nh.advertise<nav_msgs::Odometry>("absodom", 10);
      info_pub = private_nh.advertise<odom_estimator::Info>("info", 10);
      set_ignore_magnetometer_srv = private_nh.advertiseService("set_ignore_magnetometer", &NodeImpl::setIgnoreMagnetometer, this);
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
      dvl_filter.setTargetFrame(msg.header.frame_id);
      depth_filter.setTargetFrame(msg.header.frame_id);
      last_gyro = xyz2vec(msg.angular_velocity);
      local_frame_id = msg.header.frame_id;
      
      if(state && (msg.header.stamp < state->mean.t || msg.header.stamp > state->mean.t + ros::Duration(2))) {
        NODELET_ERROR("reset due to invalid stamp");
        last_mag = boost::none;
        state = boost::none;
      }
      if(state && have_gps && (last_good_gps < msg.header.stamp - ros::Duration(5.))) {
        NODELET_ERROR("reset due to no good gps data");
      }
      
      if(!state) {
        if(!last_mag) {
          std::cout << "mag missing" << std::endl;
          return;
        }
        if(have_gps) {
          if(last_good_gps && *last_good_gps > msg.header.stamp - ros::Duration(1.5) &&
                              *last_good_gps < msg.header.stamp + ros::Duration(1.5)) {
            state = init_state(msg, *last_mag, *last_good_gps_msg);
          } else {
            std::cout << "gps missing" << std::endl;
            return;
          }
        } else {
          state = init_state(msg, *last_mag, Vec<3>(start_x_ecef, start_y_ecef, start_z_ecef), Vec<3>::Zero());
        }
      } else {
        state = StateUpdater(msg)(*state);
      }
      
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //std::cout << "precov " << state->cov << std::endl << std::endl;
      //state = state->update(Vec<3>::Zero());
      
      GaussianDistribution<Vec<3>> gyro_bias_dist =
        EasyDistributionFunction<State, Vec<3>>(
          [](State const &state, Vec<0> const &) { return state.gyro_bias; }
        )(*state);
      std::cout << "gyro_bias " << gyro_bias_dist.mean.transpose()
        << " stddev: " << gyro_bias_dist.cov.diagonal().array().sqrt().transpose()
        << std::endl;
      GaussianDistribution<Vec<3>> accel_bias_dist =
        EasyDistributionFunction<State, Vec<3>>(
          [](State const &state, Vec<0> const &) { return state.accel_bias; }
        )(*state);
      std::cout << "accel_bias " << accel_bias_dist.mean.transpose()
        << " stddev: " << accel_bias_dist.cov.diagonal().array().sqrt().transpose()
        << std::endl;
      //std::cout << "grav: " << state->mean.local_g << "  " << sqrt(state->cov(State::LOCAL_G, State::LOCAL_G)) << std::endl;
      //std::cout << "ground air pressure: " << state->mean.ground_air_pressure << "  " << sqrt(state->cov(State::GROUND_AIR_PRESSURE, State::GROUND_AIR_PRESSURE)) << std::endl;
      GaussianDistribution<Vec<Dynamic>> gps_bias_dist =
        EasyDistributionFunction<State, Vec<Dynamic>>(
          [](State const &state, Vec<0> const &) { return state.gps_bias; }
        )(*state);
      for(unsigned int i = 0; i < state->mean.gps_prn.size(); i++) {
        std::cout << state->mean.gps_prn[i]
          << " " << gps_bias_dist.mean(i)
          << " " << sqrt(gps_bias_dist.cov(i, i))
          << std::endl;
      }
      std::cout << std::endl;
      

      if(state->mean.gyro_bias.norm() > .5) {
        NODELET_ERROR("reset due to bad gyro biases");
        last_mag = boost::none;
        state = boost::none;
        return;
      }

      //std::cout << "cov " << state->cov << std::endl << std::endl;
      
      {
        EasyDistributionFunction<State, Odom> transformer(
          [this, &msg](State const &state, Vec<0> const &) {
            SqMat<3> m = enu_from_ecef_mat(state.getPosECEF());
            return Odom(
              state.t,
              local_frame,
              msg.header.frame_id,
              m * state.getRelPosECEF(),
              Quaternion(m) * state.getOrientECEF(),
              state.getOrientECEF().conjugate()._transformVector(state.getVelECEF()),
              xyz2vec(msg.angular_velocity) - state.gyro_bias);
          });
        
        odom_pub.publish(msg_from_odom(transformer(*state)));
      }
      
      {
        EasyDistributionFunction<State, Odom> transformer(
          [&msg](State const &state, Vec<0> const &) {
            return Odom(
              state.t,
              "/ecef",
              msg.header.frame_id,
              state.getPosECEF(),
              state.getOrientECEF(),
              state.getOrientECEF().conjugate()._transformVector(state.getVelECEF()),
              xyz2vec(msg.angular_velocity) - state.gyro_bias);
          });
        
        absodom_pub.publish(msg_from_odom(transformer(*state)));
      }
      
      {
        odom_estimator::Info output;
        output.header.stamp = msg.header.stamp;
        
        tf::vectorEigenToMsg(gyro_bias_dist.mean, output.gyro_bias);
        tf::vectorEigenToMsg(gyro_bias_dist.cov.diagonal().array().sqrt().eval(), output.gyro_bias_stddev);
        
        tf::vectorEigenToMsg(accel_bias_dist.mean, output.accel_bias);
        tf::vectorEigenToMsg(accel_bias_dist.cov.diagonal().array().sqrt().eval(), output.accel_bias_stddev);
        
        for(unsigned int i = 0; i < state->mean.gps_prn.size(); i++) {
          output.gps_bias_prns.push_back(state->mean.gps_prn[i]);
          output.gps_bias_biases.push_back(gps_bias_dist.mean(i));
          output.gps_bias_stddevs.push_back(sqrt(gps_bias_dist.cov(i, i)));
        }
        
        info_pub.publish(output);
      }
    }
    
    void got_mag(const sensor_msgs::MagneticFieldConstPtr &msgp) {
      const sensor_msgs::MagneticField &msg = *msgp;
      
      last_mag = xyz2vec(msg.magnetic_field);
      
      if(!state) return;
      if(ignoreMagnetometer) return;
      
      SqMat<3> cov = Eigen::Map<const SqMat<3> >(msg.magnetic_field_covariance.data());
      if(cov == SqMat<3>::Zero()) {
        Vec<3> stddev(2e-7, 2e-7, 2e-7);
        stddev *= 100;
        cov = stddev.cwiseProduct(stddev).asDiagonal();
      }
      
      state = kalman_update(
        EasyDistributionFunction<State, Vec<3>, Vec<3> >(
          [&msg, this](State const &state, Vec<3> const &measurement_noise) {
            Vec<3> mag_eci = magnetic_model.getField(state.pos_eci, state.t.toSec());
            Vec<3> predicted = state.orient.conjugate()._transformVector(mag_eci) + measurement_noise;
            return predicted - xyz2vec(msg.magnetic_field);
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
          pow(.1, 2)*Vec<Dynamic>::Ones(new_prns.size()).asDiagonal())
      )(*state);
      
      state = kalman_update(
        GPSErrorObserver(msg, state->mean.gps_prn, local_gps_pos, *last_gyro),
        *state);
      
      /*if(state->mean.gps_prn.size()) {
        state = kalman_update(
          EasyDistributionFunction<State, Vec<1>, Vec<1> >(
            [](State const &state, Vec<1> const &measurement_noise) {
              return scalar_matrix(state.gps_bias.sum() / state.gps_prn.size()
                - measurement_noise(0));
            },
            GaussianDistribution<Vec<1> >(Vec<1>::Zero(), scalar_matrix(pow(.1, 2)))),
          *state);
      }*/
    }
    
    void got_dvl(const uf_common::VelocityMeasurementsConstPtr &msgp) {
      uf_common::VelocityMeasurements const & msg = *msgp;
      
      tf::StampedTransform transform;
      try {
        tf_listener.lookupTransform(local_frame_id,
          msg.header.frame_id, msg.header.stamp, transform);
      } catch (tf::TransformException ex) {
        NODELET_ERROR("Error in got_dvl: %s", ex.what());
        return;
      }
      Vec<3> local_dvl_pos; tf::vectorTFToEigen(transform.getOrigin(), local_dvl_pos);
      Quaternion local_dvl_orientation; tf::quaternionTFToEigen(transform.getRotation(), local_dvl_orientation); 
      
      if(!state) return;
      
      std::vector<uf_common::VelocityMeasurement> good;
      for(unsigned int i = 0; i < msg.velocity_measurements.size(); i++) {
        uf_common::VelocityMeasurement const & vm = msg.velocity_measurements[i];
        if(!std::isnan(vm.velocity)) {
          good.push_back(vm);
        }
      }
      
      state = kalman_update(
        EasyDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic> >(
          [&good, &local_dvl_pos, &local_dvl_orientation, this](State const &state, Vec<Dynamic> const &measurement_noise) {
            Vec<3> dvl_vel = local_dvl_orientation.inverse()._transformVector(
              state.getOrientECEF().inverse()._transformVector(
                state.getVelECEF(local_dvl_pos, *last_gyro)));
            
            Vec<Dynamic> res(good.size());
            for(unsigned int i = 0; i < good.size(); i++) {
              uf_common::VelocityMeasurement const & vm = good[i];
              res(i) = (xyz2vec(vm.direction).dot(dvl_vel) + measurement_noise(i)) - vm.velocity;
            }
            return res;
          },
          GaussianDistribution<Vec<Dynamic> >(
            Vec<Dynamic>::Zero(good.size()),
            pow(.05, 2) * Vec<Dynamic>::Ones(good.size()).asDiagonal())),
        *state);
    }
    
    void got_depth(const uf_common::Float64StampedConstPtr &msgp) {
      uf_common::Float64Stamped const & msg = *msgp;
      
      
      tf::StampedTransform transform;
      try {
        tf_listener.lookupTransform(local_frame_id,
          msg.header.frame_id, msg.header.stamp, transform);
      } catch (tf::TransformException ex) {
        NODELET_ERROR("Error in got_depth: %s", ex.what());
        return;
      }
      Vec<3> local_depth_pos; tf::vectorTFToEigen(transform.getOrigin(), local_depth_pos);
      
      if(!state) return;
      
      state = kalman_update(
        EasyDistributionFunction<State, Vec<1>, Vec<1> >(
          [&](State const &state, Vec<1> const &measurement_noise) {
            SqMat<3> m = enu_from_ecef_mat(state.getPosECEF());
            double estimated = -(m * state.getRelPosECEF())(2) + measurement_noise(0); // XXX doesn't account for local_depth_pos
            return scalar_matrix(estimated - msg.data);
          },
          GaussianDistribution<Vec<1> >(
            Vec<1>::Zero(),
            pow(.1, 2) * Vec<1>::Ones().asDiagonal())),
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
    
    bool setIgnoreMagnetometer(SetIgnoreMagnetometer::Request &request,
                               SetIgnoreMagnetometer::Response &response) {
        ignoreMagnetometer = request.ignore;
        return true;
    }
    
    
    tf::TransformListener tf_listener;
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber press_sub;
    message_filters::Subscriber<rawgps_common::Measurements> gps_sub;
    tf::MessageFilter<rawgps_common::Measurements> gps_filter;
    message_filters::Subscriber<uf_common::VelocityMeasurements> dvl_sub;
    tf::MessageFilter<uf_common::VelocityMeasurements> dvl_filter;
    message_filters::Subscriber<uf_common::Float64Stamped> depth_sub;
    tf::MessageFilter<uf_common::Float64Stamped> depth_filter;
    ros::Publisher odom_pub;
    ros::Publisher absodom_pub;
    ros::Publisher info_pub;
    
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
