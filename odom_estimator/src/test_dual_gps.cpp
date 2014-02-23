#include <functional>

#include <ros/ros.h>

#include "odom_estimator/dual_gps.h"

using namespace odom_estimator;


void callback(dual_gps::State const & state) {
  std::cout << "hi" << std::endl;
}

class GPSHandler {
  std::function<void(dual_gps::State const &)> cb;
  dual_gps::Worker w;
public:
  GPSHandler(std::function<void(dual_gps::State const &)> cb) :
    cb(cb) {
  }
  
  void handle_gps_pair(rawgps_common::Measurements const & a, rawgps_common::Measurements const & b) {
    w.handle_gps_pair(a, b);
    cb(w.opt_state_dist->mean);
  }
};

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_gps");
  
  ros::NodeHandle nh;
  
  GPSHandler gh(callback);
  GPSManager gm(GPSManager::CallbackType(GPSManager::CallbackType(std::bind(&GPSHandler::handle_gps_pair, &gh, std::placeholders::_1, std::placeholders::_2))));
  ros::Subscriber sub1 = nh.subscribe("gps1", 1, boost::function<void(rawgps_common::MeasurementsConstPtr const &)>(boost::bind(&GPSManager::got_gps1, &gm, _1)));
  ros::Subscriber sub2 = nh.subscribe("gps2", 1, boost::function<void(rawgps_common::MeasurementsConstPtr const &)>(boost::bind(&GPSManager::got_gps2, &gm, _1)));
  
  ros::spin();
  
  return 0;
}
