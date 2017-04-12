#ifndef DRIVER_H
#define DRIVER_H

#include <cmath>
#include <fstream>

#include <boost/asio/serial_port.hpp>
#include <boost/foreach.hpp>
#include <boost/none.hpp>
#include <boost/math/constants/constants.hpp>

#include <mil_tools/msg_helpers.hpp>

#include <mil_msgs/VelocityMeasurements.h>
#include <mil_msgs/RangeStamped.h>

namespace rdi_explorer_dvl {
static uint16_t getu16le(uint8_t *i) { return *i | (*(i + 1) << 8); }
static int32_t gets32le(uint8_t *i) {
  return *i | (*(i + 1) << 8) | (*(i + 2) << 16) | (*(i + 3) << 24);
}

class Device {
 private:
  typedef std::vector<boost::uint8_t> ByteVec;

  const std::string port;
  const int baudrate;
  boost::asio::io_service io;
  boost::asio::serial_port p;

  bool read_byte(uint8_t &res) {
    while (true) {
      try {
        p.read_some(boost::asio::buffer(&res, sizeof(res)));
        return true;
      } catch (const std::exception &exc) {
        ROS_ERROR("error on read: %s; reopening", exc.what());
        open();
        return false;
      }
    }
  }
  bool read_short(uint16_t &x) {
    uint8_t low;
    if (!read_byte(low)) return false;
    uint8_t high;
    if (!read_byte(high)) return false;
    x = 256 * high + low;
    return true;
  }

  void open() {
    try {
      p.close();
      p.open(port);
      p.set_option(boost::asio::serial_port::baud_rate(baudrate));
      return;
    } catch (const std::exception &exc) {
      ROS_ERROR("error on open(%s): %s; reopening after delay", port.c_str(), exc.what());
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
  }

 public:
  Device(const std::string port, int baudrate) : port(port), baudrate(baudrate), p(io) {
    // open is called on first read() in the _polling_ thread
  }

  void read(boost::optional<mil_msgs::VelocityMeasurements> &res,
            boost::optional<mil_msgs::RangeStamped> &height_res) {
    res = boost::none;
    height_res = boost::none;

    if (!p.is_open()) {
      open();
      return;
    }

    ByteVec ensemble;
    ensemble.resize(4);

    if (!read_byte(ensemble[0])) return;  // Header ID
    if (ensemble[0] != 0x7F) return;

    ros::Time stamp = ros::Time::now();

    if (!read_byte(ensemble[1])) return;  // Data Source ID
    if (ensemble[1] != 0x7F) return;

    if (!read_byte(ensemble[2])) return;  // Size low
    if (!read_byte(ensemble[3])) return;  // Size high
    uint16_t ensemble_size = getu16le(ensemble.data() + 2);
    ensemble.resize(ensemble_size);
    for (int i = 4; i < ensemble_size; i++) {
      if (!read_byte(ensemble[i])) return;
    }

    uint16_t checksum = 0;
    BOOST_FOREACH (uint16_t b, ensemble)
      checksum += b;
    uint16_t received_checksum;
    if (!read_short(received_checksum)) return;
    if (received_checksum != checksum) {
      ROS_ERROR("Invalid DVL ensemble checksum. received: %i calculated: %i size: %i",
                received_checksum, checksum, ensemble_size);
      return;
    }

    if (ensemble.size() < 6) return;
    for (int dt = 0; dt < ensemble[5]; dt++) {
      int offset = getu16le(ensemble.data() + 6 + 2 * dt);
      if (ensemble.size() - offset < 2) continue;
      uint16_t section_id = getu16le(ensemble.data() + offset);

      std::vector<double> correlations(4, nan(""));
      if (section_id == 0x5803) {  // Bottom Track High Resolution Velocity
        if (ensemble.size() - offset < 2 + 4 * 4) continue;
        res = boost::make_optional(mil_msgs::VelocityMeasurements());
        res->header.stamp = stamp;

        std::vector<geometry_msgs::Vector3> dirs;
        {
          double tilt = 30 * boost::math::constants::pi<double>() / 180;
          double x = sin(tilt);
          double z = cos(tilt);
          dirs.push_back(mil_tools::make_xyz<geometry_msgs::Vector3>(-x, 0, -z));
          dirs.push_back(mil_tools::make_xyz<geometry_msgs::Vector3>(+x, 0, -z));
          dirs.push_back(mil_tools::make_xyz<geometry_msgs::Vector3>(0, +x, -z));
          dirs.push_back(mil_tools::make_xyz<geometry_msgs::Vector3>(0, -x, -z));
        }
        for (int i = 0; i < 4; i++) {
          mil_msgs::VelocityMeasurement m;
          m.direction = dirs[i];
          int32_t vel = gets32le(ensemble.data() + offset + 2 + 4 * i);
          m.velocity = -vel * .01e-3;
          if (vel == -3276801) {  // -3276801 indicates no data
            ROS_ERROR("DVL didn't return bottom velocity for beam %i", i + 1);
            m.velocity = nan("");
          }
          res->velocity_measurements.push_back(m);
        }
      } else if (section_id == 0x0600) {  // Bottom Track
        for (int i = 0; i < 4; i++) {
          correlations[i] = *(ensemble.data() + offset + 32 + i);
        }
      } else if (section_id == 0x5804) {  // Bottom Track Range
        if (ensemble.size() - offset < 2 + 4 * 3) continue;
        if (gets32le(ensemble.data() + offset + 10) <= 0) {
          ROS_ERROR("DVL didn't return height over bottom");
          continue;
        }
        height_res = boost::make_optional(mil_msgs::RangeStamped());
        height_res->header.stamp = stamp;
        height_res->range = gets32le(ensemble.data() + offset + 10) * 0.1e-3;
      }
      if (res) {
        for (int i = 0; i < 4; i++) {
          res->velocity_measurements[i].correlation = correlations[i];
        }
      }
    }
  }

  void send_heartbeat() {
    double maxdepth = 15;

    std::stringstream buf;
    buf << "CR0\r";              // load factory settings (won't change baud rate)
    buf << "#BJ 100 110 000\r";  // enable only bottom track high res velocity and bottom track
                                 // range
    // buf << "#BK2\r"; // send water mass pings when bottom track pings fail
    // buf << "#BL7,36,46\r"; // configure near layer and far layer to 12 and 15 feet
    buf << "ES0\r";         // 0 salinity
    buf << "EX00000\r";     // no transformation
    buf << "EZ10000010\r";  // configure sensor sources. Provide manual data for everything except
                            // speed of sound and temperature
    buf << "BX" << std::setw(5) << std::setfill('0') << (int)(maxdepth * 10 + 0.5)
        << '\r';                        // configure max depth
    buf << "TT2012/03/04, 05:06:07\r";  // set RTC
    buf << "CS\r";                      // start pinging

    std::string str = buf.str();

    try {
      size_t written = 0;
      while (written < str.size()) {
        written += p.write_some(boost::asio::buffer(str.data() + written, str.size() - written));
      }
    } catch (const std::exception &exc) {
      ROS_ERROR("error on write: %s; dropping", exc.what());
    }
  }

  void abort() { p.close(); }
};
}

#endif
