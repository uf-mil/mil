#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>

ros::Publisher pub;
gazebo::common::Time currentTime;
gazebo::common::Time lastCollisionTime;

void cb(ConstContactsPtr &_contacts){

    bool back_wheel_touch_ground;
    int back_wheel_ground_count = 0;
    ros::param::get("wheel_contact/wheel_touch_ground", back_wheel_touch_ground);
    std::string contact_list = "";
    for (unsigned int i = 0; i < _contacts->contact_size(); ++i) {
        std::string indyCollisionStr1 = _contacts->contact(i).collision1();
        std::string indyCollisionStr2 = _contacts->contact(i).collision2();
        std::string indyCollisionSubStr1 =
                indyCollisionStr1.substr(0, indyCollisionStr1.find("back"));
        std::string indyCollisionSubStr2 =
                indyCollisionStr2.substr(0, indyCollisionStr2.find("back"));

        if(indyCollisionSubStr1 == "indyav_car::" || indyCollisionSubStr2 == "indyav_car::"){
           back_wheel_ground_count++;
        }

        contact_list += _contacts->contact(i).collision1() + "\n";
    }
    ros::param::set("contact_list", contact_list);

    if (back_wheel_touch_ground && back_wheel_ground_count == 0){
        ros::param::set("wheel_contact/wheel_touch_ground", false);
    } else if (!back_wheel_touch_ground && back_wheel_ground_count > 0){
        ros::param::set("wheel_contact/wheel_touch_ground", true);
    }
}

int main(int _argc, char **_argv){
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "wheel_touch_ground");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/new_smyrna/physics/contacts", cb);

    while (true)
    {
        gazebo::common::Time::MSleep(20);
        ros::spinOnce();
    }
    ros::spin();
    gazebo::client::shutdown();
}
