#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include "nv08c_driver/driver.h"


namespace nv08c_driver {
    
    class Nodelet : public nodelet::Nodelet {
        public:
            Nodelet() {}
            ~Nodelet() {
                heartbeat_timer.stop();
                running = false;
                device->abort();
                polling_thread_inst.join();
            }
            
            virtual void onInit() {
                std::string port; ROS_ASSERT_MSG(getPrivateNodeHandle().getParam("port", port),
                    "\"port\" param missing");
                int baudrate = 115200; getPrivateNodeHandle().getParam("baudrate", baudrate);
                ROS_ASSERT_MSG(getPrivateNodeHandle().getParam("frame_id", frame_id),
                    "\"frame_id\" param missing");
                
                pub = getNodeHandle().advertise<std_msgs::String>("gps_serial", 10);
                
                device = boost::make_shared<Device>(port, baudrate);
                heartbeat_timer = getNodeHandle().createTimer(ros::Duration(5), boost::bind(&Nodelet::heartbeat_callback, this, _1), true);
                running = true;
                polling_thread_inst = boost::thread(boost::bind(&Nodelet::polling_thread, this));
            }
            
        private:
            void heartbeat_callback(const ros::TimerEvent& event) {
                device->send_heartbeat();
            }
            
            void polling_thread() {
                while(running) {
                    std_msgs::String msg;
                    if(!device->read(msg.data))
                        continue;
                    //msg.header.stamp = ros::Time::now();
                    //msg.header.frame_id = frame_id;
                    pub.publish(msg);
                }
            }
            
            std::string frame_id;
            ros::Publisher pub;
            boost::shared_ptr<Device> device;
            ros::Timer heartbeat_timer;
            bool running;
            boost::thread polling_thread_inst;
    };
    
    PLUGINLIB_DECLARE_CLASS(nv08c_driver, nodelet, nv08c_driver::Nodelet, nodelet::Nodelet);
}
