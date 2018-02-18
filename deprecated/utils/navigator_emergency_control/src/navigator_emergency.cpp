#include <iostream>

#include <ros/ros.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Float64.h>

#include <signal.h>
#include <boost/foreach.hpp>
#include <string>

//#include <kill_handling/listener.h>
//#include <kill_handling/broadcaster.h>
#include <arm_bootloader/arm_bootloader.h>
#include <navigator_emergency_control/protocol.h>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>

extern unsigned char firmware_bin[];
extern int firmware_bin_len;

using namespace navigator_emergency_control;
using namespace std;

// Functionality to allow a member function to be used in place of a void
// (*func_ptr)(int)
// 		See
// http://stackoverflow.com/questions/13238050/convert-stdbind-to-function-pointer
// -------------------------------------------------------------------

// This node interprets the controller data received from the Xbee, and
// publishes it as a joy
// Note: This is Frankenstein'd from Forrest's stm32f3discovery_imu_driver.cpp
// Sorry for the mess!

template <unsigned ID, typename Functor>
boost::optional<Functor> &get_local()
{
  static boost::optional<Functor> local;
  return local;
}

template <unsigned ID, typename Functor>
typename Functor::result_type wrapper(int sig)
{
  return get_local<ID, Functor>().get()(sig);
}

template <typename ReturnType>
struct Func
{
  typedef ReturnType (*type)(int);
};

template <unsigned ID, typename Functor>
typename Func<typename Functor::result_type>::type get_wrapper(Functor f)
{
  (get_local<ID, Functor>()) = f;
  return wrapper<ID, Functor>;
}

// ----------------------------------------------------------------------

class interface
{
private:  // Typedefs
  typedef uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink>> checksum;
  typedef uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> packet;

private:  // Vars
  // Note:
  //			In the case that a class does not have a constructor
  // that
  // is
  // convinent to use
  //			in the initializer list of this class constructor a
  // pointer
  // was
  // used instead.

  // IO
  boost::asio::io_service io_svr_;
  boost::asio::serial_port sp_;

  // Random num gen
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
  boost::shared_ptr<arm_bootloader::SerialPortSink> sps_;

  // Bootloader
  arm_bootloader::Dest dest_;
  boost::shared_ptr<arm_bootloader::Reader<Response>> reader_;

  // Protocol
  boost::shared_ptr<packet> packetizer_;
  boost::shared_ptr<checksum> checksumadder_;

  // Pwms
  // double joy_;

  // Publishers and subscribers
  ros::Publisher joy_pub_;

  // TF
  std::string frame_id_;

  // Kill
  // kill_handling::KillListener kill_listener_;
  // kill_handling::KillBroadcaster kill_broadcaster_;
  // bool killed_;

private:  // Functions
  void setPwm_(double *pwm, std_msgs::Float64ConstPtr msg);
  void onShutdown_();
  void xmlrpcShutdown_(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result);
  void sigintShutdown_(int sig);

  // void onKill_();
  // void onUnkill_();

public:  // Functions
  interface(int argc, char **argv);
  void run_();
};

interface::interface(int argc, char **argv) : io_svr_(), sp_(io_svr_)
/*kill_listener_(boost::bind(&interface::onKill_, this),
boost::bind(&interface::onUnkill_, this)),
kill_broadcaster_("xbee", "Killed navigator_xbee"),
killed_(false)*/
{
  // Initialize ROS with an asynchronism spinner

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();

  // Initialize as zero
  // joy_ = (double) 0.0;

  // Setup kill signals
  // Convert from member function to void (*)(int)
  boost::function<void(int)> boost_f = boost::bind(&interface::sigintShutdown_, this, _1);
  void (*c_f)(int) = get_wrapper<1>(boost_f);
  signal(SIGINT, c_f);  // ctr-c (or any other SIGINT producer)

  ros::XMLRPCManager::instance()->unbind("shutdown");  // rosnode kill cmd line tool
  boost::function<void(XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue &)> f =
      boost::bind(&interface::xmlrpcShutdown_, this, _1, _2);
  ros::XMLRPCManager::instance()->bind("shutdown", f);

  // Setup IO
  std::string port;
  private_nh.getParam("port", port);
  sp_.open(port);
  sp_.set_option(boost::asio::serial_port::baud_rate(115200));

  std::string deststr;
  private_nh.getParam("dest", deststr);
  dest_ = strtol(deststr.c_str(), NULL, 0);

  // Get the frame id
  private_nh.getParam("frame_id", frame_id_);

  // Run the bootloader
  if (argc <= 1)
  {
    if (!arm_bootloader::attempt_bootload(port, sp_, dest_, firmware_bin, firmware_bin_len))
    {
      ROS_ERROR("bootloading failed");
      ros::shutdown();
      return;
    }
  }

  // Generate random number
  std::random_device rd;
  gen_ = std::mt19937(rd());
  dis_ = std::uniform_int_distribution<>(1, 65535);

  // Make a reader
  reader_ = boost::shared_ptr<arm_bootloader::Reader<Response>>(new arm_bootloader::Reader<Response>(sp_, 1000));

  // Setup protocol
  sps_ = boost::shared_ptr<arm_bootloader::SerialPortSink>(new arm_bootloader::SerialPortSink(port, sp_));
  packetizer_ = boost::shared_ptr<packet>(new packet(*sps_));
  checksumadder_ = boost::shared_ptr<checksum>(new checksum(*packetizer_));

  // Set up publishers and subscribers
  joy_pub_ = nh.advertise<sensor_msgs::Joy>("joy_emergency", 1, true);

  // Clear the kill in case this node previously sent a kill
  // kill_broadcaster_.clear();
}

/*void interface::onKill_()
{
  killed_ = true;

  std::string res;
  std::vector<std::string> reasons = kill_listener_.get_kills();

  BOOST_FOREACH(std::string str, reasons)
  {
    res += str;
  }

  //ROS_WARN("PWM generation killed because: %s", res.c_str());
}

void interface::onUnkill_()
{
  //ROS_WARN("PWM generation started");
  killed_ = false;
}*/

void interface::setPwm_(double *pwm, std_msgs::Float64ConstPtr msg)
{
  (*pwm) = msg->data;
}

void interface::onShutdown_()
{
  // Send out a kill msgs
  // kill_broadcaster_.send(true);

  // Stop both subscribers in case somehow a msg comes between setting pwms to
  // zero and the write pwm cmd

  // Write the zeros
  ros::Time now = ros::Time::now();

  sensor_msgs::Joy joyXbee;
  joyXbee.buttons.resize(13);
  joyXbee.axes.resize(4);
  joyXbee.axes[1] = 0.0;
  joyXbee.axes[0] = 0.0;
  joyXbee.axes[3] = 0.0;
  joyXbee.buttons[7] = 0;
  joyXbee.buttons[3] = 0;
  joyXbee.buttons[2] = 0;
  joyXbee.buttons[0] = 0;
  joyXbee.buttons[1] = 0;
  joyXbee.buttons[11] = 0;
  joyXbee.buttons[12] = 0;

  joyXbee.header.frame_id = "/base_link";
  joyXbee.header.stamp = now;
  joy_pub_.publish(joyXbee);

  // Shutdown ros
  ros::shutdown();
}

void interface::sigintShutdown_(int sig)
{
  onShutdown_();
}

// This is only called when using rosnode kill
void interface::xmlrpcShutdown_(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    num_params = params.size();
  }

  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    onShutdown_();
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void interface::run_()
{
  double autorepeat_rate_ = 0;
  double coalesce_interval_ = 0.001;

  bool tv_set = false;
  bool publication_pending = false;
  int tv_timer;

  float last_axes[4] = { 0 };
  bool last_buttons[13] = { 0 };

  while (ros::ok())
  {
    bool publish_now = false;
    bool publish_soon = false;

    Command cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.dest = dest_;
    cmd.id = dis_(gen_);
    cmd.command = CommandID::SetPWM;  // Retooled to get joy data from uP

    // cmd.args.SetPWM.length[0] = joy_;
    // cmd.args.SetPWM.length[1] = joy_;

    write_object(cmd, (*checksumadder_));

    boost::optional<Response> resp = reader_->read(cmd.id);
    if (!resp)
    {
      ROS_WARN("Timeout receiving joy packet");
    }

    ros::Time now = ros::Time::now();

    sensor_msgs::Joy joyXbee;
    joyXbee.buttons.resize(13);
    joyXbee.axes.resize(4);
    joyXbee.axes[1] = resp->resp.SetPWM.joy[0];           // Received UD data from uP
    joyXbee.axes[0] = resp->resp.SetPWM.joy[1];           // Received LR data from uP
    joyXbee.axes[3] = resp->resp.SetPWM.joy[3];           // Torque data
    joyXbee.buttons[7] = resp->resp.SetPWM.buttons[7];    //"Start"
    joyXbee.buttons[3] = resp->resp.SetPWM.buttons[3];    //"Change Mode"
    joyXbee.buttons[2] = resp->resp.SetPWM.buttons[2];    //"Kill"
    joyXbee.buttons[0] = resp->resp.SetPWM.buttons[0];    //"Station Hold"
    joyXbee.buttons[1] = resp->resp.SetPWM.buttons[1];    //"Shooter Cancel"
    joyXbee.buttons[11] = resp->resp.SetPWM.buttons[11];  //"RC Control"
    joyXbee.buttons[12] = resp->resp.SetPWM.buttons[12];  //"Auto Control"

    joyXbee.header.frame_id = "/base_link";
    joyXbee.header.stamp = now;

    // Timing for joy publishing butchered from Blaise Gassend's joy_node.cpp

    for (int i = 0; i < 13; i++)
    {
      if (joyXbee.buttons[i] != last_buttons[i])
      {
        publish_now = true;
      }
      last_buttons[i] = joyXbee.buttons[i];
    }

    for (int i = 0; i < 4; i++)
    {
      if (joyXbee.axes[i] != last_axes[i])
      {
        publish_soon = true;
      }
      last_axes[i] = joyXbee.axes[i];
    }

    if (tv_set && tv_timer < 1)  // Assume that the timer has expired.
      publish_now = true;

    if (publish_now)
    {
      // joy_pub_.publish(joyXbee);
      publish_now = false;
      tv_set = false;
      publication_pending = false;
      publish_soon = false;
    }

    if (!publication_pending && publish_soon)
    {
      tv_timer = 10;
      publication_pending = true;
      tv_set = true;
    }

    // timer for autorepeat.
    if (!tv_set && autorepeat_rate_ > 0)
    {
      tv_timer = int(1 / autorepeat_rate);
      tv_set = true;
    }

    tv_timer--;
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xbee", ros::init_options::NoSigintHandler);

  interface node(argc, argv);

  node.run_();

  return EXIT_SUCCESS;
}
