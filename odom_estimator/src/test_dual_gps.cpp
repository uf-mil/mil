#include <functional>

#include <ros/ros.h>

#include "odom_estimator/dual_gps.h"

using namespace odom_estimator;


class GPSManager {
public:
  typedef std::function<void(rawgps_common::Measurements const &, rawgps_common::Measurements const &)> CallbackType;
private:
  CallbackType cb;
  std::list<rawgps_common::Measurements> gps1_measurements;
  std::list<rawgps_common::Measurements> gps2_measurements;
public:
  GPSManager(CallbackType const & cb) :
    cb(cb) {
  }
  
  void got_gps1(rawgps_common::MeasurementsConstPtr const &msgp) {
    rawgps_common::Measurements const & msg = *msgp;
    assert(gps1_measurements.empty() || msg.sync > gps1_measurements.front().sync);
    gps1_measurements.push_back(msg);
    think();
  }
  void got_gps2(rawgps_common::MeasurementsConstPtr const &msgp) {
    rawgps_common::Measurements const & msg = *msgp;
    assert(gps2_measurements.empty() || msg.sync > gps2_measurements.front().sync);
    gps2_measurements.push_back(msg);
    think();
  }
  
  void think() {
    while(!gps1_measurements.empty() && !gps2_measurements.empty()) {
      if(gps1_measurements.front().sync < gps2_measurements.front().sync) {
        gps1_measurements.pop_front();
      } else if(gps1_measurements.front().sync > gps2_measurements.front().sync) {
        gps2_measurements.pop_front();
      } else {
        assert(gps1_measurements.front().sync == gps2_measurements.front().sync);
        cb(gps1_measurements.front(), gps2_measurements.front());
        gps1_measurements.pop_front();
        gps2_measurements.pop_front();
      }
    }
  }
};


class Node {
private:
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  GPSManager gm;
  dual_gps::Worker w;
  ros::Publisher pub;
  
  void handle_gps_pair(rawgps_common::Measurements const & a, rawgps_common::Measurements const & b) {
    std::set<int> good_prns = dual_gps::get_good_prns(a, b);
    
    std::cout << a.header.frame_id << " - " << b.header.frame_id << std::endl;
    
    double m = 0;
    BOOST_FOREACH(int prn, good_prns) {
      rawgps_common::Satellite const & a_sat = dual_gps::get_sat(a, prn);
      rawgps_common::Satellite const & b_sat = dual_gps::get_sat(b, prn);
      m += (a_sat.carrier_distance - b_sat.carrier_distance)/good_prns.size();
    }
    
    BOOST_FOREACH(int prn, good_prns) {
      rawgps_common::Satellite const & a_sat = dual_gps::get_sat(a, prn);
      rawgps_common::Satellite const & b_sat = dual_gps::get_sat(b, prn);
      std::cout << "prn " << prn << "> carrier_difference: " << a_sat.carrier_distance - b_sat.carrier_distance - m << " direction: " << xyz2vec(a_sat.direction_enu).transpose() << std::endl;
    }
    std::cout << std::endl;
    
    w.handle_gps_pair(a, b);
    
    std::cout << "relpos_enu: " << w.opt_state_dist->mean.relpos_enu.transpose() << " stddev: " << w.opt_state_dist->cov.block<3,3>(0,0).diagonal().array().sqrt().transpose() << std::endl;
    std::cout << "relvel_enu: " << w.opt_state_dist->mean.relvel_enu.transpose() << " stddev: " << w.opt_state_dist->cov.block<3,3>(3,3).diagonal().array().sqrt().transpose() << std::endl;
    std::cout << "centerpos_enu: " << w.opt_state_dist->mean.centerpos_enu.transpose() << " stddev: " << w.opt_state_dist->cov.block<3,3>(6,6).diagonal().array().sqrt().transpose() << std::endl;
    { int i = 0; BOOST_FOREACH(int prn, w.opt_state_dist->mean.gps_prn) {
      std::cout << prn << ": " << w.opt_state_dist->mean.getGPSBias(prn) << " stddev: " << sqrt(w.opt_state_dist->cov(9+i,9+i)) << std::endl;
    i++; } }
    std::cout << w.opt_state_dist->mean.gps_bias.transpose() << std::endl;
    
    GaussianDistribution<AngleManifold> yaw = EasyDistributionFunction<dual_gps::State, AngleManifold>(
      [](dual_gps::State const & state, Vec<0> const &) {
        Vec<3> v = state.relpos_enu - state.centerpos_enu;
        return atan2(v(1), v(0));
      })(*w.opt_state_dist);
    std::cout << "yaw:" << yaw.mean << " stddev: " << sqrt(yaw.cov(0,0)) << std::endl;
    
    {
      geometry_msgs::PointStamped msg;
      msg.header.stamp = a.header.stamp;
      msg.header.frame_id = "/enu";
      tf::pointEigenToMsg(w.opt_state_dist->mean.relpos_enu - w.opt_state_dist->mean.centerpos_enu, msg.point);
      pub.publish(msg);
    }
    
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    //std::cout << "hi" << w.opt_state_dist->mean.relpos_enu.transpose() << " vel: " << w.opt_state_dist->mean.relvel_enu.transpose() << std::endl;
  }
public:
  Node(ros::NodeHandle & nh) :
    gm(GPSManager::CallbackType(std::bind(&Node::handle_gps_pair, this, std::placeholders::_1, std::placeholders::_2))) {
    pub = nh.advertise<geometry_msgs::PointStamped>("rel_gps_pos", 10);
    sub1 = nh.subscribe("gps1", 1, boost::function<void(rawgps_common::MeasurementsConstPtr const &)>(boost::bind(&GPSManager::got_gps1, &gm, _1)));
    sub2 = nh.subscribe("gps2", 1, boost::function<void(rawgps_common::MeasurementsConstPtr const &)>(boost::bind(&GPSManager::got_gps2, &gm, _1)));
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_gps");
  
  ros::NodeHandle nh;
  
  Node n(nh);
  
  ros::spin();
  
  return 0;
}
