#include <bits/stdint-uintn.h>
#include <endian.h>
#include <mil_msgs/DepthStamped.h>
#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <limits>
#include <mutex>
#include <thread>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

// modeled after mil_passive_sonar/sylphase_ros_bridge

using tcp = boost::asio::ip::tcp;

class NavTubeDriver
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_;

  std::string ip_;
  int port_;
  std::string frame_id_;
  uint16_t hz_;

  uint64_t acceptable_frequency;

  ros::Time prev;

  std::thread timer_thread;

  std::mutex m;

  bool running = true;

  bool initialized = false;

  static const uint8_t sync1 = 0x37;
  static const uint8_t sync2 = 0x01;

  uint8_t heartbeat_packet[2 + sizeof(hz_)];

  boost::shared_ptr<tcp::socket> connect();

  void send_heartbeat(boost::shared_ptr<tcp::socket> socket);

  void read_messages(boost::shared_ptr<tcp::socket> socket);

  double calculate_pressure(uint16_t analog_input);

  double yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, ang_rate_x, ang_rate_y, ang_rate_z;

public:
  NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);

  ~NavTubeDriver();

  void run();
};

NavTubeDriver::NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
{
  pub_ = nh.advertise<mil_msgs::DepthStamped>("depth", 10);
  ip_ = private_nh.param<std::string>("ip", std::string("192.168.37.61"));
  port_ = private_nh.param<int>("port", 33056);
  frame_id_ = private_nh.param<std::string>("frame_id", "/depth");
  imu_frame_id_ = private_nh.param<std::string>("imu_frame_id", "/imu");
  mag_frame_id_ = private_nh.param<std::string>("mag_frame_id", "/mag");

  int hz__ = private_nh.param<int>("hz", 20);

  if (hz__ > std::numeric_limits<uint16_t>::max())
  {
    ROS_WARN_STREAM("Depth polling frequency is greater than 16 bits!");
  }

  hz_ = hz__;

  acceptable_frequency = (1'000'000'000 * 1.25) / (uint64_t)hz_;

  heartbeat_packet[0] = sync1;
  heartbeat_packet[1] = sync2;
  uint16_t nw_ordering = htons(hz_);
  reinterpret_cast<uint16_t*>(&heartbeat_packet[2])[0] = nw_ordering;
}

NavTubeDriver::~NavTubeDriver()
{
  {
    std::lock_guard<std::mutex> lock(m);
    running = false;
  }
  timer_thread.join();
}

boost::shared_ptr<tcp::socket> NavTubeDriver::connect()
{
  using ip_address = boost::asio::ip::address;
  tcp::endpoint endpoint(ip_address::from_string(ip_), port_);

  ROS_INFO_STREAM("Connecting to Depth Server");
  boost::asio::io_service io_service;
  boost::shared_ptr<tcp::socket> socket = boost::make_shared<tcp::socket>(io_service);
  socket->connect(endpoint);
  ROS_INFO_STREAM("Connection to Depth Server established");

  return socket;
}

void NavTubeDriver::run()
{
  while (ros::ok())
  {
    try
    {
      prev = ros::Time::now();
      boost::shared_ptr<tcp::socket> socket;

      socket = connect();
      timer_thread = std::thread(&NavTubeDriver::send_heartbeat, this, socket);
      initialized = true;
      read_messages(socket);
    }
    catch (boost::system::system_error const& e)
    {
      ros::Duration wait_time(5);
      ROS_WARN_STREAM("Error with NavTube Depth driver TCP socket " << e.what() << ". Trying again in "
                                                                    << wait_time.toSec() << " seconds");

      if (initialized)
        timer_thread.join();
      initialized = false;
      wait_time.sleep();
    }
  }
}

void NavTubeDriver::send_heartbeat(boost::shared_ptr<tcp::socket> socket)
{
  try
  {
    while (true)
    {
      boost::asio::write(*socket, boost::asio::buffer(heartbeat_packet));

      {
        std::lock_guard<std::mutex> lock(m);
        if (!running)
          return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  catch (boost::system::system_error const& e)
  {
  }
}

void NavTubeDriver::read_messages(boost::shared_ptr<tcp::socket> socket)
{
  mil_msgs::DepthStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.seq = 0;

  sensor_msgs::Imu msgIMU msgIMU.header.frame_id = imu_frame_id_;
  msgIMU.header.seq = 0;

  sensor_msgs::MagneticField msgMag msgMag.header.frame_id = imu_frame_id_;
  msgMag.header.seq = 0;

  uint8_t backing[10];

  auto buffer = boost::asio::buffer(backing, sizeof(backing));

  while (ros::ok())
  {
    if (ros::Time::now().toNSec() - prev.toNSec() > acceptable_frequency)
    {
      ROS_WARN_STREAM("Depth sampling rate is falling behind.");
    }

    if (!boost::asio::buffer_size(buffer))
    {
      // Bytes are out of sync so try and resync
      if (backing[0] != sync1 || backing[1] != sync2)
      {
        for (int i = 0; i < (sizeof(backing) / sizeof(backing[0])) - 1; i++)
        {
          backing[i] = backing[i + 1];
        }
        buffer = boost::asio::buffer(backing + (sizeof(backing) / sizeof(backing[0])) - sizeof(backing[0]),
                                     sizeof(backing[0]));
      }
      else
      {
        uint64_t bits = be64toh(*reinterpret_cast<uint64_t*>(&backing[3]));
        double value = *reinterpret_cast<double*>(&bits);

        if (&backing[2] == 0x00)
        {
          ++msg.header.seq;
          msg.header.stamp = ros::Time::now();
          msg.depth = value;
          pub_.publish(msg);
        }
        else
        {
          // Map the byte data to the value
          switch (&backing[2])
          {
            case 0x01:
              yaw = value break;

            case 0x02:
              pitch = value break;

            case 0x03:
              roll = value break;

            case 0x04:
              mag_x = value break;

            case 0x05:
              mag_y = value break;

            case 0x06:
              mag_z = value break;

            case 0x07:
              accel_x = value break;

            case 0x08:
              accel_y = value break;

            case 0x09:
              accel_z = value break;

            case 0x0a:
              ang_rate_x = value break;

            case 0x0b:
              ang_rate_y = value break;

            case 0x0c:
              ang_rate_z = value break;

            default:
              ROS_WARN_STREAM("Unexpected data hex id...");
              break;
          }

          // Publish IMU message

          // Publish magnetometer message
        }

        buffer = boost::asio::buffer(backing, sizeof(backing));
      }
    }

    size_t bytes_read = socket->read_some(buffer);

    buffer = boost::asio::buffer(buffer + bytes_read);
    prev = ros::Time::now();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_tube_driver");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  NavTubeDriver node(nh, private_nh);
  node.run();
}
