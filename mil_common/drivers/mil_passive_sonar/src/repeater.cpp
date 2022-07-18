/* ROS node to read in the TCP stream produced by the Sylphase passive sonar board
 * and publish it to ROS as a mil_passive_sonar/HydrophoneSamples message
 */
#include <mil_passive_sonar/HydrophoneSamplesStamped.h>
#include <ros/ros.h>
#include <boost/asio.hpp>

/// Class representing the node. Could easily be made into a Nodelet
class Repeater
{
public:


  Repeater(ros::NodeHandle nh){
    this->nh = nh;
    pub = nh.advertise<mil_passive_sonar::HydrophoneSamplesStamped>("/hydrophones/samples_raw", 1);
    sub = nh.subscribe("/hydrophones/samples", 1000, &Repeater::cb, this);
  };

  void cb(const mil_passive_sonar::HydrophoneSamplesStamped& msg) {
    
    mil_passive_sonar::HydrophoneSamplesStamped new_msg;
    new_msg.header = msg.header;
    new_msg.hydrophone_samples.channels = msg.hydrophone_samples.channels;
    new_msg.hydrophone_samples.sample_rate = msg.hydrophone_samples.sample_rate;
    new_msg.hydrophone_samples.samples = msg.hydrophone_samples.samples;
    
    for (int i = 0; i < msg.hydrophone_samples.data.size(); ++i)
    {
      //std::cout << new_msg.hydrophone_samples.data[i] << std::endl;
      //new_msg.hydrophone_samples.data.push_back( msg.hydrophone_samples.data[i] );
      new_msg.hydrophone_samples.data.push_back( (msg.hydrophone_samples.data[i] << 8) |
                                      ((msg.hydrophone_samples.data[i] >> 8) & 0x00ff) );
      //std::cout << new_msg.hydrophone_samples.data[i] << std::endl<< std::endl;
    }
    pub.publish(new_msg);
  }
  

private:

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "repeater");

  ros::NodeHandle nh;
  Repeater node(nh);
  ros::spin();
}
