#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/utility/in_place_factory.hpp> 

#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <mil_msgs/DepthStamped.h>
#include <mil_msgs/VelocityMeasurements.h>

#include <odom_estimator/Info.h>
#include <odom_estimator/SetIgnoreMagnetometer.h>

#include "odom_estimator/earth.h"
#include "odom_estimator/kalman.h"
#include "odom_estimator/magnetic.h"
#include "odom_estimator/odometry.h"
#include "odom_estimator/state.h"
#include "odom_estimator/unscented_transform.h"
#include "odom_estimator/util.h"

namespace odom_estimator
{
static magnetic::MagneticModel const magnetic_model(ros::package::getPath("odom_estimator") + "/data/WMM.COF");

GaussianDistribution<State> init_state(sensor_msgs::Imu const &msg, Vec<3> last_mag, Vec<3> pos_ecef, Vec<3> vel_ecef,
                                       Vec<3> rel_pos_ecef)
{
  Vec<3> pos_eci = inertial_from_ecef(msg.header.stamp.toSec(), pos_ecef);
  Vec<3> vel_eci = inertial_vel_from_ecef_vel(msg.header.stamp.toSec(), vel_ecef, pos_eci);

  Vec<3> mag_eci = magnetic_model.getField(pos_eci, msg.header.stamp.toSec());
  Vec<3> predicted_acc_eci = inertial_acc_from_ecef_acc(msg.header.stamp.toSec(), Vec<3>::Zero(), pos_eci);
  Vec<3> predicted_accelerometer_eci = predicted_acc_eci - gravity::gravity(pos_eci);
  Vec<3> accel_body = xyz2vec(msg.linear_acceleration);
  Quaternion orient_eci = triad(predicted_accelerometer_eci, mag_eci, accel_body, last_mag);

  Vec<State::RowsAtCompileTime> stdev = (Vec<State::RowsAtCompileTime>(18) << 100, 100, 100, 100, 100, 100, .05, .05,
                                         .05, 10, 10, 10, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2)
                                            .finished();
  SqMat<State::RowsAtCompileTime> tmp = stdev.asDiagonal();

  return GaussianDistribution<State>(State(msg.header.stamp, msg.header.stamp, pos_eci,
                                           inertial_from_ecef(msg.header.stamp.toSec(), rel_pos_ecef), orient_eci,
                                           vel_eci, Vec<3>::Zero(), Vec<3>::Zero()),
                                     tmp * tmp);
}

class NodeImpl
{
private:
  boost::function<const std::string &()> getName;
  ros::NodeHandle &nh;
  ros::NodeHandle &private_nh;
  double start_x_ecef, start_y_ecef, start_z_ecef;
  std::string local_frame;
  ros::ServiceServer set_ignore_magnetometer_srv;
  bool ignoreMagnetometer;
  Vec<3> last_rel_pos_ecef_;

public:
  NodeImpl(boost::function<const std::string &()> getName, ros::NodeHandle *nh_, ros::NodeHandle *private_nh_)
    : getName(getName)
    , nh(*nh_)
    , private_nh(*private_nh_)
    , local_frame("/enu")
    , ignoreMagnetometer(false)
    , mag_sub(nh, "imu/mag", 1)
    , mag_filter(mag_sub, tf_listener, "", 10)
    , dvl_sub(nh, "dvl", 1)
    , dvl_filter(dvl_sub, tf_listener, "", 10)
    , depth_sub(nh, "depth", 1)
    , depth_filter(depth_sub, tf_listener, "", 10)
    , last_mag(boost::none)
    , last_good_dvl(boost::none)
    , state(boost::none)
  {
    private_nh.getParam("start_x_ecef", start_x_ecef);
    private_nh.getParam("start_y_ecef", start_y_ecef);
    private_nh.getParam("start_z_ecef", start_z_ecef);
    private_nh.getParam("local_frame", local_frame);

    imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 10, boost::bind(&NodeImpl::got_imu, this, _1));
    mag_filter.registerCallback(boost::bind(&NodeImpl::got_mag, this, _1));
    dvl_filter.registerCallback(boost::bind(&NodeImpl::got_dvl, this, _1));
    depth_filter.registerCallback(boost::bind(&NodeImpl::got_depth, this, _1));
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    absodom_pub = nh.advertise<nav_msgs::Odometry>("absodom", 10);
    info_pub = private_nh.advertise<odom_estimator::Info>("info", 10);
    set_ignore_magnetometer_srv =
        private_nh.advertiseService("set_ignore_magnetometer", &NodeImpl::setIgnoreMagnetometer, this);

    last_rel_pos_ecef_ = Vec<3>::Zero();
  }

private:
  void got_imu(const sensor_msgs::ImuConstPtr &msgp)
  {
    // const sensor_msgs::Imu &msg = *msgp;
    sensor_msgs::Imu msg = *msgp;
    Eigen::Map<SqMat<3>>(msg.angular_velocity_covariance.data()) = pow(0.02, 2) * SqMat<3>::Identity();
    Eigen::Map<SqMat<3>>(msg.linear_acceleration_covariance.data()) = pow(0.06, 2) * SqMat<3>::Identity();

    mag_filter.setTargetFrame(msg.header.frame_id);
    dvl_filter.setTargetFrame(msg.header.frame_id);
    depth_filter.setTargetFrame(msg.header.frame_id);
    last_gyro = xyz2vec(msg.angular_velocity);
    local_frame_id = msg.header.frame_id;

    if (state && (msg.header.stamp < state->mean.t || msg.header.stamp > state->mean.t + ros::Duration(2)))
    {
      NODELET_ERROR("reset due to invalid stamp");
      last_mag = boost::none;
      state = boost::none;
    }

    if (!state)
    {
      if (!last_mag)
      {
        std::cout << "mag missing" << std::endl;
        return;
      }
      if (last_good_dvl && *last_good_dvl > msg.header.stamp - ros::Duration(1.5) &&
          *last_good_dvl < msg.header.stamp + ros::Duration(1.5))
      {
        state = init_state(msg, *last_mag, Vec<3>(start_x_ecef, start_y_ecef, start_z_ecef), Vec<3>::Zero(),
                           last_rel_pos_ecef_);
      }
      else
      {
        std::cout << "dvl missing" << std::endl;
        return;
      }
    }
    else
    {
      state = StateUpdater(msg)(*state);
    }

    GaussianDistribution<Vec<3>> gyro_bias_dist = EasyDistributionFunction<State, Vec<3>>(
        [](State const &state, Vec<0> const &) { return state.gyro_bias; })(*state);
    GaussianDistribution<Vec<3>> accel_bias_dist = EasyDistributionFunction<State, Vec<3>>(
        [](State const &state, Vec<0> const &) { return state.accel_bias; })(*state);

    if (state->mean.gyro_bias.norm() > .5)
    {
      NODELET_ERROR("reset due to bad gyro biases");
      last_mag = boost::none;
      state = boost::none;
      return;
    }

    last_rel_pos_ecef_ = state->mean.getRelPosECEF();

    {
      EasyDistributionFunction<State, Odom> transformer([this, &msg](State const &state, Vec<0> const &) {
        SqMat<3> m = enu_from_ecef_mat(state.getPosECEF());
        return Odom(state.t, local_frame, msg.header.frame_id, m * state.getRelPosECEF(),
                    Quaternion(m) * state.getOrientECEF(),
                    state.getOrientECEF().conjugate()._transformVector(state.getVelECEF()),
                    xyz2vec(msg.angular_velocity) - state.gyro_bias);
      });

      odom_pub.publish(msg_from_odom(transformer(*state)));
    }

    {
      EasyDistributionFunction<State, Odom> transformer([&msg](State const &state, Vec<0> const &) {
        return Odom(state.t, "/ecef", msg.header.frame_id, state.getPosECEF(), state.getOrientECEF(),
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

      info_pub.publish(output);
    }
  }

  void got_mag(const sensor_msgs::MagneticFieldConstPtr &msgp)
  {
    const sensor_msgs::MagneticField &msg = *msgp;

    tf::StampedTransform transform;
    try
    {
      tf_listener.lookupTransform(local_frame_id, msg.header.frame_id, msg.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      NODELET_ERROR("Error in got_mag: %s", ex.what());
      return;
    }
    Quaternion local_mag_orientation;
    tf::quaternionTFToEigen(transform.getRotation(), local_mag_orientation);

    last_mag = xyz2vec(msg.magnetic_field);

    if (!state)
      return;
    if (ignoreMagnetometer)
      return;

    SqMat<3> cov = Eigen::Map<const SqMat<3>>(msg.magnetic_field_covariance.data());
    if (cov == SqMat<3>::Zero())
    {
      Vec<3> stddev(2e-7, 2e-7, 2e-7);
      stddev *= 100;
      cov = stddev.cwiseProduct(stddev).asDiagonal();
    }

    Vec<3> mag_eci = magnetic_model.getField(state->mean.pos_eci, state->mean.t.toSec());
    state = kalman_update(
        EasyDistributionFunction<State, Vec<1>, Vec<3>>(
            [&msg, &mag_eci, &local_mag_orientation, this](State const &state, Vec<3> const &measurement_noise) {
              SqMat<3> enu_from_ecef = enu_from_ecef_mat(state.getPosECEF());
              Vec<3> predicted = state.orient.conjugate()._transformVector(mag_eci) +
                                 local_mag_orientation._transformVector(measurement_noise);
              Vec<3> predicted_enu = enu_from_ecef * state.getOrientECEF()._transformVector(predicted);
              double predicted_angle = atan2(predicted_enu(1), predicted_enu(0));
              Vec<3> measured_enu = enu_from_ecef *
                                    state.getOrientECEF()._transformVector(
                                        local_mag_orientation._transformVector(xyz2vec(msg.magnetic_field)));
              double measured_angle = atan2(measured_enu(1), measured_enu(0));
              double error_angle = measured_angle - predicted_angle;
              double pi = boost::math::constants::pi<double>();
              while (error_angle < -pi)
                error_angle += 2 * pi;
              while (error_angle > pi)
                error_angle -= 2 * pi;
              return scalar_matrix(error_angle);
            },
            GaussianDistribution<Vec<3>>(Vec<3>::Zero(), cov)),
        *state);
  }

  void got_dvl(const mil_msgs::VelocityMeasurementsConstPtr &msgp)
  {
    mil_msgs::VelocityMeasurements const &msg = *msgp;

    tf::StampedTransform transform;
    try
    {
      tf_listener.lookupTransform(local_frame_id, msg.header.frame_id, msg.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      NODELET_ERROR("Error in got_dvl: %s", ex.what());
      return;
    }
    Vec<3> local_dvl_pos;
    tf::vectorTFToEigen(transform.getOrigin(), local_dvl_pos);
    Quaternion local_dvl_orientation;
    tf::quaternionTFToEigen(transform.getRotation(), local_dvl_orientation);

    std::vector<mil_msgs::VelocityMeasurement> good;
    for (unsigned int i = 0; i < msg.velocity_measurements.size(); i++)
    {
      mil_msgs::VelocityMeasurement const &vm = msg.velocity_measurements[i];
      if (!std::isnan(vm.velocity))
      {
        good.push_back(vm);
      }
    }

    if (good.size() >= 3)
    {
      last_good_dvl = msg.header.stamp;
      std::cout << "got dvl" << std::endl;
    }
    else
    {
      std::cout << "bad dvl" << std::endl;
    }

    if (!state)
      return;

    state = kalman_update(
        EasyDistributionFunction<State, Vec<Dynamic>, Vec<Dynamic>>(
            [&good, &local_dvl_pos, &local_dvl_orientation, this](State const &state,
                                                                  Vec<Dynamic> const &measurement_noise) {
              Vec<3> dvl_vel = local_dvl_orientation.inverse()._transformVector(
                  state.getOrientECEF().inverse()._transformVector(state.getVelECEF(local_dvl_pos, *last_gyro)));

              Vec<Dynamic> res(good.size());
              for (unsigned int i = 0; i < good.size(); i++)
              {
                mil_msgs::VelocityMeasurement const &vm = good[i];
                res(i) = (xyz2vec(vm.direction).dot(dvl_vel) + measurement_noise(i)) - vm.velocity;
              }
              return res;
            },
            GaussianDistribution<Vec<Dynamic>>(Vec<Dynamic>::Zero(good.size()),
                                               pow(.05, 2) * Vec<Dynamic>::Ones(good.size()).asDiagonal())),
        *state);
  }

  void got_depth(const mil_msgs::DepthStampedConstPtr &msgp)
  {
    mil_msgs::DepthStamped const &msg = *msgp;

    tf::StampedTransform transform;
    try
    {
      tf_listener.lookupTransform(local_frame_id, msg.header.frame_id, msg.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      NODELET_ERROR("Error in got_depth: %s", ex.what());
      return;
    }
    Vec<3> local_depth_pos;
    tf::vectorTFToEigen(transform.getOrigin(), local_depth_pos);

    if (!state)
      return;

    state = kalman_update(EasyDistributionFunction<State, Vec<1>, Vec<1>>(
                              [&](State const &state, Vec<1> const &measurement_noise) {
                                SqMat<3> m = enu_from_ecef_mat(state.getPosECEF());
                                double estimated =
                                    -(m * state.getRelPosECEF(local_depth_pos))(2) + measurement_noise(0);
                                return scalar_matrix(estimated - msg.depth);
                              },
                              GaussianDistribution<Vec<1>>(Vec<1>::Zero(), pow(.1, 2) * Vec<1>::Ones().asDiagonal())),
                          *state);
  }

  bool setIgnoreMagnetometer(SetIgnoreMagnetometer::Request &request, SetIgnoreMagnetometer::Response &response)
  {
    ignoreMagnetometer = request.ignore;
    return true;
  }

  tf::TransformListener tf_listener;
  ros::Subscriber imu_sub;
  message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub;
  tf::MessageFilter<sensor_msgs::MagneticField> mag_filter;
  message_filters::Subscriber<mil_msgs::VelocityMeasurements> dvl_sub;
  tf::MessageFilter<mil_msgs::VelocityMeasurements> dvl_filter;
  message_filters::Subscriber<mil_msgs::DepthStamped> depth_sub;
  tf::MessageFilter<mil_msgs::DepthStamped> depth_filter;
  ros::Publisher odom_pub;
  ros::Publisher absodom_pub;
  ros::Publisher info_pub;

  boost::optional<Vec<3>> last_mag;
  boost::optional<ros::Time> last_good_dvl;
  boost::optional<GaussianDistribution<State>> state;
  boost::optional<Vec<3>> last_gyro;
  std::string local_frame_id;
};

class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet()
  {
  }

  virtual void onInit()
  {
    nodeimpl = boost::in_place(boost::bind(&Nodelet::getName, this), &getNodeHandle(), &getPrivateNodeHandle());
  }

private:
  boost::optional<NodeImpl> nodeimpl;
};
PLUGINLIB_EXPORT_CLASS( odom_estimator::Nodelet, nodelet::Nodelet);
}
