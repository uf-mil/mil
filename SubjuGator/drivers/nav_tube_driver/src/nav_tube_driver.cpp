#include <bits/stdint-uintn.h>
#include <endian.h>
#include <mil_msgs/DepthStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <limits>
#include <mutex>
#include <thread>
#include <iostream>

// modeled after mil_passive_sonar/sylphase_ros_bridge

using tcp = boost::asio::ip::tcp;

class NavTubeDriver
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher depth_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry recent_odom_msg_;

  std::string ip_;
  int depth_port_;
  int imu_port_;
  std::string depth_frame_id_;
  std::string imu_frame_id_;
  std::string mag_frame_id_;
  uint16_t hz_;

  uint64_t acceptable_frequency;

  ros::Time prev;

  std::thread timer_thread;
  std::thread imu_thread;

  std::mutex m;

  bool running = true;

  bool initialized = false;

  static const uint8_t sync1 = 0x37;
  static const uint8_t sync2 = 0x01;

  uint8_t heartbeat_packet[2 + sizeof(hz_)];

  boost::shared_ptr<tcp::socket> connect_to_depth();
  boost::shared_ptr<tcp::socket> connect_to_imu();

  void send_heartbeat(boost::shared_ptr<tcp::socket> socket);

  void read_messages(boost::shared_ptr<tcp::socket> socket);
  void process_imu(boost::shared_ptr<tcp::socket> socket);

  double calculate_pressure(uint16_t analog_input);

public:
  NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);

  ~NavTubeDriver();

  void run();
  void odom_callback(const nav_msgs::OdometryConstPtr& msg);
};

NavTubeDriver::NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
{
  depth_pub_ = nh.advertise<mil_msgs::DepthStamped>("depth", 10);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &NavTubeDriver::odom_callback, this);
  ip_ = private_nh.param<std::string>("ip", std::string("192.168.37.61"));
  depth_port_ = private_nh.param<int>("depth_port", 33056);
  imu_port_ = private_nh.param<int>("imu_port", 33057);
  depth_frame_id_ = private_nh.param<std::string>("frame_id", "depth");
  imu_frame_id_ = private_nh.param<std::string>("imu_frame_id", "imu");
  mag_frame_id_ = private_nh.param<std::string>("mag_frame_id", "imu");

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

void NavTubeDriver::odom_callback(const nav_msgs::OdometryConstPtr& ptr)
{
  recent_odom_msg_ = *ptr;
}

NavTubeDriver::~NavTubeDriver()
{
  {
    std::lock_guard<std::mutex> lock(m);
    running = false;
  }
  timer_thread.join();
  imu_thread.join();
}

boost::shared_ptr<tcp::socket> NavTubeDriver::connect_to_depth()
{
  using ip_address = boost::asio::ip::address;
  tcp::endpoint endpoint(ip_address::from_string(ip_), depth_port_);

  ROS_INFO_STREAM("Connecting to Depth Server");
  boost::asio::io_service io_service;
  boost::shared_ptr<tcp::socket> socket = boost::make_shared<tcp::socket>(io_service);
  socket->connect(endpoint);
  ROS_INFO_STREAM("Connection to Depth Server established");

  return socket;
}

boost::shared_ptr<tcp::socket> NavTubeDriver::connect_to_imu()
{
  using ip_address = boost::asio::ip::address;
  tcp::endpoint endpoint(ip_address::from_string(ip_), imu_port_);

  ROS_INFO_STREAM("Connecting to IMU Server");
  boost::asio::io_service io_service;
  boost::shared_ptr<tcp::socket> socket = boost::make_shared<tcp::socket>(io_service);
  socket->connect(endpoint);
  ROS_INFO_STREAM("Connection to IMU Server established");

  return socket;
}

void NavTubeDriver::run()
{
  while (ros::ok())
  {
    try
    {
      prev = ros::Time::now();
      boost::shared_ptr<tcp::socket> depth_socket;
      boost::shared_ptr<tcp::socket> imu_socket;

      depth_socket = connect_to_depth();
      imu_socket = connect_to_imu();
      timer_thread = std::thread(&NavTubeDriver::send_heartbeat, this, depth_socket);
      imu_thread = std::thread(&NavTubeDriver::process_imu, this, imu_socket);
      initialized = true;
      read_messages(depth_socket);
    }
    catch (boost::system::system_error const& e)
    {
      ros::Duration wait_time(5);
      ROS_WARN_STREAM("Error with NavTube Depth driver TCP socket " << e.what() << ". Trying again in "
                                                                    << wait_time.toSec() << " seconds");

      if (initialized)
      {
        timer_thread.join();
        imu_thread.join();
      }
      initialized = false;
      wait_time.sleep();
    }
  }
}

void NavTubeDriver::process_imu(boost::shared_ptr<tcp::socket> socket)
{
  sensor_msgs::Imu msg;
  msg.header.frame_id = imu_frame_id_;
  msg.header.seq = 0;

  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.frame_id = mag_frame_id_;
  mag_msg.header.seq = 0;

  // 2 sync chars, 13 doubles
  uint8_t backing[2 * sizeof(char) + 13 * sizeof(double)];
  auto buffer = boost::asio::buffer(backing, sizeof(backing));

  while (ros::ok())
  {
    if (ros::Time::now().toNSec() - prev.toNSec() > acceptable_frequency)
    {
      ROS_WARN_STREAM("IMU sampling rate is falling behind.");
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
        ++msg.header.seq;
        msg.header.stamp = ros::Time::now();

        ++mag_msg.header.seq;
        mag_msg.header.stamp = ros::Time::now();

        // Value order is documented in uf-mil/navigation-tube/vn100.py
        double values[13];
        memcpy(values, backing + 2, sizeof(values));

        for (int i = 0; i < 13; i++)
        {
          uint64_t bits = be64toh(*reinterpret_cast<uint64_t*>(&values[i]));
          values[i] = *reinterpret_cast<double*>(&bits);
        }

        double x = values[0];
        double y = values[1];
        double z = values[2];
        double w = values[3];

        tf2::Quaternion orientation_quaternion(x, y, z, w);
        msg.orientation = tf2::toMsg(orientation_quaternion);

        msg.linear_acceleration_covariance[0] =
                msg.linear_acceleration_covariance[4] = 
                msg.linear_acceleration_covariance[8] = 0.1;

        msg.angular_velocity.x = values[10];
        msg.angular_velocity.y = values[11];
        msg.angular_velocity.z = values[12];

        msg.angular_velocity_covariance[0] =
                msg.angular_velocity_covariance[4] = 
                msg.angular_velocity_covariance[8] = 0.1;

        msg.linear_acceleration.x = values[7];
        msg.linear_acceleration.y = values[8];
        msg.linear_acceleration.z = values[9];

        msg.linear_acceleration_covariance[0] =
                msg.linear_acceleration_covariance[4] = 
                msg.linear_acceleration_covariance[8] = 0.1;

        imu_pub_.publish(msg);

        // Publish mag
        mag_msg.magnetic_field.x = values[4];
        mag_msg.magnetic_field.y = values[5];
        mag_msg.magnetic_field.z = values[6];

        mag_msg.magnetic_field_covariance[0] =
                msg.linear_acceleration_covariance[4] = 
                msg.linear_acceleration_covariance[8] = 0.1;

        mag_pub_.publish(mag_msg);

        buffer = boost::asio::buffer(backing, sizeof(backing));
      }
    }

    std::cout << "attempting to read... at " << ros::Time::now() << std::endl;
    size_t bytes_read = socket->read_some(buffer);
    // if (ec == boost::asio::error::eof) {
    //     ROS_WARN_THROTTLE(3, "Cannot read data from the IMU through the navtube!");
    // }

    buffer = boost::asio::buffer(buffer + bytes_read);
    prev = ros::Time::now();

    ros::spinOnce();
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
  msg.header.frame_id = depth_frame_id_;
  msg.header.seq = 0;

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
        ++msg.header.seq;
        msg.header.stamp = ros::Time::now();

        uint64_t bits = be64toh(*reinterpret_cast<uint64_t*>(&backing[2]));
        double pressure = *reinterpret_cast<double*>(&bits);
        if (recent_odom_msg_.header.seq)
        {
          // Accounts for the dynamic pressure applied to the pressure sensor
          // when the sub is moving forwards or backwards
          double velocity = recent_odom_msg_.twist.twist.linear.x;
          double vel_effect = (abs(velocity) * velocity) / (1000 * 9.81);
          msg.depth = pressure + vel_effect;
        }
        else
        {
          msg.depth = pressure;
        }

        depth_pub_.publish(msg);
        buffer = boost::asio::buffer(backing, sizeof(backing));
      }
    }

    size_t bytes_read = socket->read_some(buffer);

    buffer = boost::asio::buffer(buffer + bytes_read);
    prev = ros::Time::now();

    ros::spinOnce();
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
