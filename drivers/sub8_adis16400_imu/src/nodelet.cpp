#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <mil_tools/param_helpers.hpp>

#include "adis16400_imu/driver.h"

namespace adis16400_imu {

class Nodelet : public nodelet::Nodelet {
 public:
  Nodelet() {}
  ~Nodelet() {
    running = false;
    polling_thread_inst.join();
  }

  virtual void onInit() {
    std::string port = mil_tools::getParam<std::string>(getPrivateNodeHandle(), "port");
    frame_id = mil_tools::getParam<std::string>(getPrivateNodeHandle(), "frame_id");
    drop_every_ = mil_tools::getParam<unsigned int>(getPrivateNodeHandle(), "divide", 1);

    ros::NodeHandle& nh = getNodeHandle();
    pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag_raw", 10);

    count = 0;
    running = true;
    device = boost::make_shared<Device>(port);
    polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
  }

 private:
  void polling_thread() {
    while (running) {
      sensor_msgs::Imu imu;
      sensor_msgs::MagneticField mag;
      if (device->read(frame_id, imu, mag)) {
        if (count++ % drop_every_ != 0) continue;
        pub.publish(imu);
        mag_pub.publish(mag);
      }
    }
  }

  boost::shared_ptr<Device> device;
  std::string frame_id;
  ros::Publisher pub;
  ros::Publisher mag_pub;
  bool running;
  boost::thread polling_thread_inst;
  int count;
  unsigned int drop_every_;
};

PLUGINLIB_DECLARE_CLASS(adis16400_imu, nodelet, adis16400_imu::Nodelet, nodelet::Nodelet);
}
