#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/range/adaptor/map.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "nv08c_driver/Packet.h"
#include "nv08c_driver/PacketSet.h"
#include "nv08c_driver/driver.h"


namespace nv08c_driver {
    
    class PersistentPacketPattern {
        private:
            uint8_t id;
            std::vector<uint8_t> data_prefix;
            uint8_t data_key_length;
        public:
            PersistentPacketPattern(uint8_t id, std::vector<uint8_t> data_prefix, uint8_t data_key_length) :
                id(id), data_prefix(data_prefix), data_key_length(data_key_length) { }
            boost::optional<std::vector<uint8_t> > matches(Packet packet) const {
                if(packet.id != id) return boost::none;
                if(packet.data.size() < data_prefix.size() + data_key_length) return boost::none;
                for(unsigned int i = 0; i < data_prefix.size(); i++)
                    if(packet.data[i] != data_prefix[i]) return boost::none;
                std::vector<uint8_t> key;
                key.push_back(id);
                key.insert(key.end(), data_prefix.begin(), data_prefix.end());
                for(unsigned int i = data_prefix.size(); i < data_prefix.size() + data_key_length; i++)
                    key.push_back(packet.data[i]);
                return key;
            }
    };
    
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
                persistent_patterns.push_back(PersistentPacketPattern(0x4A, std::vector<uint8_t>({}), 0)); // Ionosphere Parameters
                persistent_patterns.push_back(PersistentPacketPattern(0x4B, std::vector<uint8_t>({}), 0)); // GPS, GLONASS and UTC Time Scales Parameters
                persistent_patterns.push_back(PersistentPacketPattern(0x70, std::vector<uint8_t>({}), 0)); // Software Version
                persistent_patterns.push_back(PersistentPacketPattern(0xF6, std::vector<uint8_t>({}), 0)); // Geocentric Coordinates of Antenna
                persistent_patterns.push_back(PersistentPacketPattern(0xF7, std::vector<uint8_t>({1}), 1)); // Ephemeris for GPS satellite
                persistent_patterns.push_back(PersistentPacketPattern(0xF7, std::vector<uint8_t>({2}), 2)); // Ephemeris for GLONASS satellite
                
                std::string port; ROS_ASSERT_MSG(getPrivateNodeHandle().getParam("port", port),
                    "\"port\" param missing");
                int baudrate = 115200; getPrivateNodeHandle().getParam("baudrate", baudrate);
                ROS_ASSERT_MSG(getPrivateNodeHandle().getParam("frame_id", frame_id),
                    "\"frame_id\" param missing");
                
                pub = ros::NodeHandle(getNodeHandle(), "nv08c_serial").advertise<nv08c_driver::Packet>("ephemeral", 10);
                latch_pub = ros::NodeHandle(getNodeHandle(), "nv08c_serial").advertise<nv08c_driver::PacketSet>("persistent", 10, true);
                
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
                    boost::optional<std::pair<uint8_t, std::vector<uint8_t> > > packet_data =
                        device->read_packet();
                    if(!packet_data)
                        continue;
                    
                    nv08c_driver::Packet packet;
                    packet.header.stamp = ros::Time::now();
                    packet.header.frame_id = frame_id;
                    packet.id = packet_data->first;
                    packet.data = packet_data->second;
                    
                    std::cout << "received packet. ID: " << (int)packet.id << std::endl;
                    bool was_persistent = false;
                    BOOST_FOREACH(const PersistentPacketPattern &pattern, persistent_patterns) {
                        boost::optional<std::vector<uint8_t> > match_res = pattern.matches(packet);
                        std::cout << "    match " << &pattern - persistent_patterns.data() << " " << static_cast<bool>(match_res) << std::endl;
                        if(match_res) {
                            std::cout << "received match: " << (int)packet_data->first << std::endl;
                            persistent_packets[*match_res] = packet;
                            
                            nv08c_driver::PacketSet packet_set;
                            BOOST_FOREACH(const Packet &other_packet, persistent_packets | boost::adaptors::map_values)
                                packet_set.packets.push_back(other_packet);
                            latch_pub.publish(packet_set);
                            
                            was_persistent = true;
                            break;
                        }
                    }
                    if(was_persistent) {
                        continue;
                    }
                    
                    //std::cout << "received: " << (int)packet_data->first << " not persistant" << std::endl;
                    // not a persistent message
                    pub.publish(packet);
                }
            }
            
            std::vector<PersistentPacketPattern> persistent_patterns;
            std::map<std::vector<uint8_t>, nv08c_driver::Packet> persistent_packets;
            std::string frame_id;
            ros::Publisher pub;
            ros::Publisher latch_pub;
            boost::shared_ptr<Device> device;
            ros::Timer heartbeat_timer;
            bool running;
            boost::thread polling_thread_inst;
    };
    
    PLUGINLIB_DECLARE_CLASS(nv08c_driver, nodelet, nv08c_driver::Nodelet, nodelet::Nodelet);
}
