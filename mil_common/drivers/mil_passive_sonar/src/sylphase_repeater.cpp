/* ROS node to read in the TCP stream produced by the Sylphase passive sonar board
 * and publish it to ROS as a mil_passive_sonar/HydrophoneSamples message
 */
#include <arpa/inet.h>
#include <mil_passive_sonar/HydrophoneSamplesStamped.h>
#include <ros/ros.h>

/// Class representing the node. Could easily be made into a Nodelet
class SylphaseRepeater
{
public:

  SylphaseRepeater(ros::NodeHandle nh, ros::NodeHandle private_nh);


private:
  /// Wait forever reading messages and publishing that
  void repeat(const mil_passive_sonar::HydrophoneSamplesStamped& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

SylphaseRepeater::SylphaseRepeater(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh), private_nh_(private_nh)
{
  pub_ = nh.advertise<mil_passive_sonar::HydrophoneSamplesStamped>("/hydrophones/samples_r", 1);
  sub_ = nh.subscribe("/hydrophones/samples", 1000, &SylphaseRepeater::repeat, this);
}

void SylphaseRepeater::repeat(const mil_passive_sonar::HydrophoneSamplesStamped& msg)
{

  // Pre-allocate message
  mil_passive_sonar::HydrophoneSamplesStamped new_msg = msg;
  /*msg.header.frame_id = frame_id_;
  msg.header.seq = 0;
  msg.hydrophone_samples.channels = CHANNELS;
  msg.hydrophone_samples.samples = SAMPLES_TO_CAPTURE_PER_CHANNEL;
  msg.hydrophone_samples.sample_rate = SAMPLES_PER_SECOND;
  msg.hydrophone_samples.data.resize(SAMPLES_TO_CAPTURE);*/

  // Received packets are Big-Endian, convert to system
  for (size_t i = 0; i < new_msg.hydrophone_samples.data.size(); ++i)
  {
    new_msg.hydrophone_samples.data[i] = ntohs(new_msg.hydrophone_samples.data[i]);
  }

  pub_.publish(new_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sylphase_repeater");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  SylphaseRepeater node(nh, private_nh);
  ros::spin();
}