#include <map>

#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_datatypes.h>

#include "magnetic_dynamic_compensation/FieldInfo.h"

namespace magnetic_dynamic_compensation
{
class Nodelet : public nodelet::Nodelet
{
private:
  ros::Subscriber sub_fieldinfo;
  ros::Subscriber sub;
  ros::Publisher pub;

  std::map<std::string, FieldInfo> fieldinfo_map;

  void handle_fieldinfo(const FieldInfo::ConstPtr& msg)
  {
    if (fieldinfo_map.count(msg->id) && msg->header.stamp < fieldinfo_map[msg->id].header.stamp)
    {
      return;  // ignore out of order messages
    }
    fieldinfo_map[msg->id] = *msg;
  }

  void handle(const sensor_msgs::MagneticField::ConstPtr& msg)
  {
    tf::Vector3 measured_mag_field;
    tf::vector3MsgToTF(msg->magnetic_field, measured_mag_field);

    tf::Vector3 local_mag_field(0, 0, 0);
    BOOST_FOREACH (const FieldInfo& info, fieldinfo_map | boost::adaptors::map_values)
    {
      if (info.header.stamp + info.lifetime < msg->header.stamp)
      {
        continue;  // fieldinfo has expired
      }

      if (info.header.frame_id != msg->header.frame_id)
      {
        ROS_ERROR("FieldInfo's frame_id != msg's frame_id! ignoring FieldInfo %s", info.id.c_str());
        continue;
      }

      tf::Vector3 this_mag_field;
      tf::vector3MsgToTF(info.magnetic_field, this_mag_field);

      local_mag_field += this_mag_field;
    }

    tf::Vector3 real_mag_field = measured_mag_field - local_mag_field;

    sensor_msgs::MagneticField result;
    result.header.frame_id = msg->header.frame_id;
    result.header.stamp = msg->header.stamp;
    tf::vector3TFToMsg(real_mag_field, result.magnetic_field);

    pub.publish(result);
  }

public:
  Nodelet()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();

    sub_fieldinfo =
        nh.subscribe<FieldInfo>("/imu/mag_generated_info", 1000, boost::bind(&Nodelet::handle_fieldinfo, this, _1));
    sub = nh.subscribe<sensor_msgs::MagneticField>("imu/mag_raw", 1000, boost::bind(&Nodelet::handle, this, _1));
    pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  }
};

PLUGINLIB_DECLARE_CLASS(magnetic_dynamic_compensation, nodelet, magnetic_dynamic_compensation::Nodelet,
                        nodelet::Nodelet);
}
