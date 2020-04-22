#include <indyav_control/SteeringStamped.h>
#include <ros/ros.h>

template <class MSG>
class SimulatedSteeringDriver
{
public:
  SimulatedSteeringDriver(ros::NodeHandle* _nh, const std::string& sub_topic);

protected:
  ros::NodeHandle* nh_;
  ros::Subscriber sub_;
  std::map<std::string, ros::Publisher> pubs_;

  void Callback(const MSG& _msg);
};
