#include <bits/stdint-uintn.h>
#include <endian.h>
#include <math.h>
#include <mil_msgs/DepthStamped.h>
#include <ros/ros.h>
#include <stdint.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <limits>
#include <mutex>
#include <thread>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float64.h"

// modeled after mil_passive_sonar/sylphase_ros_bridge
// #define M_PI 3.14159265358979323846  /* M_PI */

using tcp = boost::asio::ip::tcp;

class NavTubeDriver
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_, pubIMU, pubMag;

  ros::Subscriber dvl_vx_sub;

  std::string ip_;
  int port_;
  std::string frame_id_;
  std::string imu_frame_id_;
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

  double yaw = -999, pitch = -999, roll = -999, mag_x = -999, mag_y = -999, mag_z = -999, accel_x = -999,
         accel_y = -999, accel_z = -999, ang_rate_x = -999, ang_rate_y = -999, ang_rate_z = -999;

  // Reset Cooldown after sudden spike
  int sequence_pen = 5;
  int yaw_pen = 0, pitch_pen = 0, roll_pen = 0, mag_x_pen = 0, mag_y_pen = 0, mag_z_pen = 0, accel_x_pen = 0,
      accel_y_pen = 0, accel_z_pen = 0, ang_rate_x_pen = 0, ang_rate_y_pen = 0, ang_rate_z_pen = 0;

  double x_orientation, y_orientation, z_orientation, w_orientation;

  double mag_threshold = 0.1;

  double accel_threshold = 1.5;

  double ang_rate_threshold = 0.3;

  float x_vel = 0;

  void set_orientation(double yaw, double pitch, double roll);

  void vel_cb(const std_msgs::Float64::ConstPtr& msg);

public:
  NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);

  ~NavTubeDriver();

  void run();
};

NavTubeDriver::NavTubeDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
{
  pub_ = nh.advertise<mil_msgs::DepthStamped>("depth", 10);
  pubIMU = nh.advertise<sensor_msgs::Imu>("py/IMU", 1000);
  pubMag = nh.advertise<sensor_msgs::MagneticField>("py/Mag", 1000);
  dvl_vx_sub = nh.subscribe("dvlVX", 10, &NavTubeDriver::vel_cb, this);

  ip_ = private_nh.param<std::string>("ip", std::string("192.168.37.61"));
  port_ = private_nh.param<int>("port", 33056);
  frame_id_ = private_nh.param<std::string>("frame_id", "/depth");
  imu_frame_id_ = private_nh.param<std::string>("imu_frame_id", "/imu");

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

void NavTubeDriver::vel_cb(const std_msgs::Float64::ConstPtr& msg)
{
  float x_vel = msg->data;
}

void NavTubeDriver::set_orientation(double yaw, double pitch, double roll)
{
  // Convert degrees to radians
  double yaw_rad = yaw * M_PI / 180.0;
  double pitch_rad = pitch * M_PI / 180.0;
  double roll_rad = roll * M_PI / 180.0;

  // Calculate trigonometric values
  double cy = cos(yaw_rad * 0.5);
  double sy = sin(yaw_rad * 0.5);
  double cp = cos(pitch_rad * 0.5);
  double sp = sin(pitch_rad * 0.5);
  double cr = cos(roll_rad * 0.5);
  double sr = sin(roll_rad * 0.5);

  w_orientation = cr * cp * cy + sr * sp * sy;
  x_orientation = sr * cp * cy - cr * sp * sy;
  y_orientation = cr * sp * cy + sr * cp * sy;
  z_orientation = cr * cp * sy - sr * sp * cy;
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
      ROS_INFO_STREAM("Reading data now");
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

  sensor_msgs::Imu msgIMU;
  msgIMU.header.frame_id = imu_frame_id_;
  msgIMU.header.seq = 0;

  sensor_msgs::MagneticField msgMag;
  msgMag.header.frame_id = imu_frame_id_;
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

        // uint8_t typeBits = be64toh(*reinterpret_cast<uint8_t*>(&backing[2]));
        int type = backing[2];
	double b2 = backing[2];
	double b0 = backing[0];
	double b1 = backing[1];
        // int type = doubType;
	
	// ROS_INFO_STREAM("Data received from PI:" + std::to_string(b0) + ", " + std::to_string(b1) + ", "+ std::to_string(b2) + ", " + std::to_string(value));

        if (type == 0)
        {
          ++msg.header.seq;
          msg.header.stamp = ros::Time::now();

          // Adjust depth accordingly
          double depth = ((value - 14.7) * 6894.76 + 0.5 * 1000 * x_vel * abs(x_vel)) / (1000 * 9.79286);

          msg.depth = depth;

          pub_.publish(msg);
        }
        else
        {
          // Map the byte data to the value
          switch (type)
          {
            case 1:
              yaw = value;
              break;

            case 2:
              pitch = value;
              break;

            case 3:
              roll = value;
              break;

            case 4:
              if (mag_x_pen > 1)
              {
                mag_x_pen--;
                break;
              }
              if (abs(value - mag_x) < mag_threshold || mag_x == -999 ||
                  mag_x_pen == 1)  // Check if change is not above threshold, check initial state, check penalty
                                   // cooldown
              {
                mag_x = value;
                mag_x_pen == 0;
              }
              else
              {
                mag_x_pen = sequence_pen;
              }
              break;

            case 5:
              if (mag_y_pen > 1)
              {
                mag_y_pen--;
                break;
              }
              if (abs(value - mag_y) < mag_threshold || mag_y == -999 || mag_y_pen == 1)
              {
                mag_y = value;
                mag_y_pen = 0;
              }
              else
              {
                mag_y_pen = sequence_pen;
              }
              break;

            case 6:
              if (mag_z_pen > 1)
              {
                mag_z_pen--;
                break;
              }
              if (abs(value - mag_z) < mag_threshold || mag_z == -999 || mag_z_pen == 1)
              {
                mag_z = value;
                mag_z_pen = 0;
              }
              else
              {
                mag_z_pen = sequence_pen;
              }
              break;

            case 7:  // put accel x readings from IMU into accel z
              if (accel_x_pen > 1)
              {
                accel_x_pen--;
                break;
              }
              if (abs(value - accel_x) < accel_threshold || accel_x == -999 || accel_x_pen == 1)
              {
                accel_x = value;
                accel_x_pen = 0;
              }
              else
              {
                accel_x_pen = sequence_pen;
              }
              break;

            case 8:  // accel y
              if (accel_y_pen > 1)
              {
                accel_y_pen--;
                break;
              }
              if (abs(value - accel_y) < accel_threshold || accel_y == -999 || accel_y_pen == 1)
              {
                accel_y = -value;
                accel_y_pen = 0;
              }
              else
              {
                accel_y_pen = sequence_pen;
              }
              break;

            case 9:  // put accel z readings from IMU into accel x
              if (accel_z_pen > 1)
              {
                accel_z_pen--;
                break;
              }
              if (abs(value - accel_z) < accel_threshold || accel_z == -999 || accel_z_pen == 1)
              {
                accel_z = value;
                accel_z_pen = 0;
                break;
              }
              else
              {
                accel_z_pen = sequence_pen;
              }
              break;

            case 10:
              if (ang_rate_x_pen > 1)
              {
                ang_rate_x_pen--;
                break;
              }
              if (abs(value - ang_rate_x) < ang_rate_threshold || ang_rate_x == -999 || ang_rate_x_pen == 1)
              {
                ang_rate_x = value;
                ang_rate_x_pen = 0;
              }
              else
              {
                ang_rate_x_pen = sequence_pen;
              }
              break;

            case 11:
              if (ang_rate_y_pen > 1)
              {
                ang_rate_y_pen--;
                break;
              }
              if (abs(value - ang_rate_y) < ang_rate_threshold || ang_rate_y == -999 || ang_rate_y_pen == 1)
              {
                ang_rate_y = value;
                ang_rate_y_pen = 0;
              }
              else
              {
                ang_rate_y_pen = sequence_pen;
              }
              break;

            case 12:
              if (ang_rate_z_pen > 1)
              {
                ang_rate_z_pen--;
                break;
              }
              if (abs(value - ang_rate_z) < ang_rate_threshold || ang_rate_z == -999 || ang_rate_z_pen == 1)
              {
                ang_rate_z = value;
                ang_rate_z_pen = 0;
              }
              else
              {
                ang_rate_z_pen = sequence_pen;
              }
              break;

            default:
              ROS_WARN_STREAM("Unexpected data hex id:" + std::to_string(type));
              break;
          }
          if (yaw == -999 || pitch == -999 || roll == -999 || mag_x == -999 || mag_y == -999 || mag_z == -999 ||
              accel_x == -999 || accel_y == -999 || accel_z == -999 || ang_rate_x == -999 || ang_rate_y == -999 ||
              ang_rate_z == -999)
          {
            ROS_INFO_STREAM("Awaiting all IMU data...");
          }
          else
          {
            set_orientation(yaw, pitch, roll);
            ++msgIMU.header.seq;
            ++msgMag.header.seq;

            msgIMU.header.stamp = ros::Time::now();
            msgMag.header.stamp = ros::Time::now();

            // Publish Raw IMU message
            msgIMU.orientation.w = w_orientation;
            msgIMU.orientation.x = x_orientation;
            msgIMU.orientation.y = y_orientation;
            msgIMU.orientation.z = z_orientation;
            msgIMU.orientation_covariance[0] = 0.01;
            msgIMU.orientation_covariance[4] = 0.01;
            msgIMU.orientation_covariance[8] = 0.01;
            msgIMU.linear_acceleration.x = accel_x;
            msgIMU.linear_acceleration.y = accel_y;
            msgIMU.linear_acceleration.z = accel_z;
            msgIMU.linear_acceleration_covariance[0] = 0.01;
            msgIMU.linear_acceleration_covariance[4] = 0.01;
            msgIMU.linear_acceleration_covariance[8] = 0.01;
            msgIMU.angular_velocity.x = ang_rate_x;
            msgIMU.angular_velocity.y = ang_rate_y;
            msgIMU.angular_velocity.z = ang_rate_z;
            msgIMU.angular_velocity_covariance[0] = 0.01;
            msgIMU.angular_velocity_covariance[4] = 0.01;
            msgIMU.angular_velocity_covariance[8] = 0.01;

            pubIMU.publish(msgIMU);

            // Publish Raw Mag Data
            msgMag.magnetic_field.x = mag_x;
            msgMag.magnetic_field.y = mag_y;
            msgMag.magnetic_field.z = mag_z;

            pubMag.publish(msgMag);
          }
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
