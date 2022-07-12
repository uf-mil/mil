#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <mil_tools/param_helpers.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "rdi_explorer_dvl/driver.hpp"

namespace rdi_explorer_dvl
{
class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet()
  {
  }
  ~Nodelet()
  {
    heartbeat_timer.stop();
    running = false;
    device->abort();
    polling_thread_inst.join();
  }

  virtual void onInit()
  {
    std::string port = mil_tools::getParam<std::string>(getPrivateNodeHandle(), "port");

    int baudrate = mil_tools::getParam<int>(getPrivateNodeHandle(), "baudrate", 115200);

    frame_id = mil_tools::getParam<std::string>(getPrivateNodeHandle(), "frame_id");

    pub = getNodeHandle().advertise<mil_msgs::VelocityMeasurements>("dvl", 10);
    range_pub = getNodeHandle().advertise<mil_msgs::RangeStamped>("dvl/range", 10);

    device = boost::make_shared<Device>(port, baudrate);
    heartbeat_timer =
        getNodeHandle().createTimer(ros::Duration(0.5), boost::bind(&Nodelet::heartbeat_callback, this, _1));
    running = true;
    polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
  }

private:
  void heartbeat_callback(const ros::TimerEvent& event)
  {
    device->send_heartbeat();
  }

  void polling_thread()
  {
    while (running)
    {
      boost::optional<mil_msgs::VelocityMeasurements> msg;
      boost::optional<mil_msgs::RangeStamped> range_msg;
      device->read(msg, range_msg);
      if (msg)
      {
        msg->header.frame_id = frame_id;
        pub.publish(*msg);
      }
      if (range_msg)
      {
        range_msg->header.frame_id = frame_id;
        range_pub.publish(*range_msg);
      }
    }
  }

  std::string frame_id;
  ros::Publisher pub;
  ros::Publisher range_pub;
  boost::shared_ptr<Device> device;
  ros::Timer heartbeat_timer;
  bool running;
  boost::thread polling_thread_inst;
};

PLUGINLIB_EXPORT_CLASS(rdi_explorer_dvl::Nodelet, nodelet::Nodelet);
}  // namespace rdi_explorer_dvl
