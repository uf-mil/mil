/* ROS node to read in the TCP stream produced by the Sylphase passive sonar board
 * and publish it to ROS as a mil_passive_sonar/HydrophoneSamples message
 */
#include <arpa/inet.h>
#include <mil_passive_sonar/HydrophoneSamples.h>
#include <ros/ros.h>
#include <boost/asio.hpp>

/// Class representing the node. Could easily be made into a Nodelet
class SylphaseSonarToRosNode
{
public:
  /// Sampling frequency of the Sylphase board
  const size_t SAMPLES_PER_SECOND = 1.2E6;
  /// Number of channels in the Sylphase board
  const size_t CHANNELS = 4;

  SylphaseSonarToRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

  /// Wait forever streaming messages
  void run();

private:
  /// Connect to the socket
  boost::asio::ip::tcp::socket connect();
  /// Wait forever reading messages and publishing that
  void read_messages(boost::asio::ip::tcp::socket& socket);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  mil_passive_sonar::HydrophoneSamples msg;
  std::string frame_id_;
  std::string ip_;
  int port_;
  double seconds_per_message_;
};

void SylphaseSonarToRosNode::run()
{
  while (ros::ok())
  {
    try
    {
      // Connect to Sylphase TCP socket
      auto socket = connect();
      // Read messages infinitely
      read_messages(socket);
    }
    catch (boost::system::system_error const& e)
    {
      // On exception, print error, wait 5 seconds, and start over
      ros::Duration wait_time(5);
      ROS_WARN_STREAM("Error with Sylphase sonar TCP socket " << e.what() << ". Trying again in " << wait_time.toSec()
                                                              << " seconds");
      wait_time.sleep();
      continue;
    }
  }
}

SylphaseSonarToRosNode::SylphaseSonarToRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh), private_nh_(private_nh)
{
  pub_ = nh.advertise<mil_passive_sonar::HydrophoneSamples>("samples", 1);
  ip_ = private_nh.param<std::string>("ip", std::string("127.0.0.1"));
  port_ = private_nh.param<int>("port", 10001);
  frame_id_ = private_nh.param<std::string>("frame", "hydrophones");
  seconds_per_message_ = private_nh.param<double>("seconds_to_capture", 0.1);
}

boost::asio::ip::tcp::socket SylphaseSonarToRosNode::connect()
{
  using ip_address = boost::asio::ip::address;
  using tcp = boost::asio::ip::tcp;
  tcp::endpoint endpoint(ip_address::from_string(ip_.c_str()), port_);
  boost::asio::io_service io_service;
  tcp::socket socket(io_service);
  socket.connect(endpoint);
  return socket;
}

void SylphaseSonarToRosNode::read_messages(boost::asio::ip::tcp::socket& socket)
{
  const size_t SAMPLES_TO_CAPTURE_PER_CHANNEL = seconds_per_message_ * SAMPLES_PER_SECOND;
  const size_t SAMPLES_TO_CAPTURE = CHANNELS * SAMPLES_TO_CAPTURE_PER_CHANNEL;
  const size_t BYTES_TO_CAPTURE = sizeof(uint16_t) * SAMPLES_TO_CAPTURE;

  // Pre-allocate message
  mil_passive_sonar::HydrophoneSamples msg;
  msg.header.frame_id = frame_id_;
  msg.header.seq = 0;
  msg.channels = CHANNELS;
  msg.samples = SAMPLES_TO_CAPTURE_PER_CHANNEL;
  msg.sample_rate = SAMPLES_PER_SECOND;
  msg.data.resize(SAMPLES_TO_CAPTURE);

  auto buffer = boost::asio::buffer(msg.data);
  bool first_packet = true;

  while (ros::ok())
  {
    // If the buffer is now full, ship the message off
    if (!boost::asio::buffer_size(buffer))
    {
      // Received packets are Big-Endian, convert to system
      for (size_t i = 0; i < msg.data.size(); ++i)
      {
        msg.data[i] = ntohs(msg.data[i]);
      }
      ++msg.header.seq;
      pub_.publish(msg);

      // Reset the buffer to the start of the vector and reset first_packet
      buffer = boost::asio::buffer(msg.data);
      first_packet = true;
    }

    // Read up to the remaining size of the buffer or how ever many are available in the socket
    size_t bytes_read = socket.read_some(buffer);
    // If this is the first packet for this message, store now as the message time
    // as it will be close to the beginning
    if (first_packet)
    {
      msg.header.stamp = ros::Time::now();
      first_packet = false;
    }

    // Move the buffer forward by the number of bytes read
    buffer = boost::asio::buffer(buffer + bytes_read);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node_name");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  SylphaseSonarToRosNode node(nh, private_nh);
  node.run();
}