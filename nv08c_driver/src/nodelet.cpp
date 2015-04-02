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
    
    class Nodelet : public nodelet::Nodelet {
        public:
            Nodelet() {}
            ~Nodelet() {
                if(!device) return;
                heartbeat_timer.stop();
                running = false;
                device->abort();
                polling_thread_inst.join();
            }
            
            virtual void onInit() {
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0x4A) return std::vector<uint8_t>{p.id}; return boost::none; }); // Ionosphere Parameters
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0x4B) return std::vector<uint8_t>{p.id}; return boost::none; }); // GPS, GLONASS and UTC Time Scales Parameters
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0x70) return std::vector<uint8_t>{p.id}; return boost::none; }); // Software Version
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0xF6) return std::vector<uint8_t>{p.id}; return boost::none; }); // Geocentric Coordinates of Antenna
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0xF7 && p.data.size() >= 2 && p.data[0] == 1) return std::vector<uint8_t>{p.id, 0, p.data[1]}; return boost::none; }); // Ephemeris for GPS satellite
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0xF7 && p.data.size() >= 3 && p.data[0] == 2) return std::vector<uint8_t>{p.id, 1, p.data[1], p.data[2]}; return boost::none; }); // Ephemeris for GLONASS satellite
                persistent_patterns.push_back([](Packet p) -> boost::optional<std::vector<uint8_t> > { if(p.id == 0xE5 && p.data.size() >= 8 && p.data[0] == 1 && p.data[2] == 2) return std::vector<uint8_t>{p.id, p.data[3], p.data[8+6]&0b111}; return boost::none; }); // bit information
                
                std::string port; if(!getPrivateNodeHandle().getParam("port", port)) {
                    throw std::runtime_error("param port required");
                }
                int baudrate = 115200; getPrivateNodeHandle().getParam("baudrate", baudrate);
                
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
            
            void handle_packet(std::pair<uint8_t, std::vector<uint8_t> > const & packet_data) {
                nv08c_driver::Packet packet;
                packet.header.stamp = ros::Time::now();
                packet.id = packet_data.first;
                packet.data = packet_data.second;
                
                std::cout << "received packet. ID: " << (int)packet.id << std::endl;
                bool was_persistent = false;
                BOOST_FOREACH(const std::function<boost::optional<std::vector<uint8_t> >(Packet)> &pattern, persistent_patterns) {
                    boost::optional<std::vector<uint8_t> > match_res = pattern(packet);
                    //std::cout << "    match " << &pattern - persistent_patterns.data() << " " << static_cast<bool>(match_res) << std::endl;
                    if(match_res) {
                        std::cout << "received match: " << (int)packet_data.first << std::endl;
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
                    return;
                }
                
                //std::cout << "received: " << (int)packet_data.first << " not persistant" << std::endl;
                // not a persistent message
                pub.publish(packet);
            }
            
            void polling_thread() {
                while(running) {
                    boost::optional<std::pair<uint8_t, std::vector<uint8_t> > > packet_data =
                        device->read_packet();
                    if(!packet_data)
                        continue;
                    
                    // split up bit information packets so they can be correctly cached
                    if(packet_data->first == 0xE5 && packet_data->second.size() >= 1) {
                        unsigned int pos = 1;
                        for(unsigned int i = 0; i < packet_data->second[0]; i++) {
                            unsigned int start = pos;
                            if(packet_data->second.size() < pos + 7) { std::cout << "early end" << std::endl; break; }
                            unsigned int type = packet_data->second[pos + 1]; pos += 7;
                            unsigned int length;
                            if(type == 1) length = 12;
                            else if(type == 2) length = 40;
                            else if(type == 4) length = 32;
                            else {
                                std::cout << "unknown SNS type in bit information packet" << std::endl;
                                break;
                            }
                            if(packet_data->second.size() < pos + length) { std::cout << "early end" << std::endl; break; }
                            pos += length;
                            std::pair<uint8_t, std::vector<uint8_t> > p2;
                            p2.first = 0xE5;
                            p2.second.resize(1 + 7 + length);
                            p2.second[0] = 1;
                            for(unsigned int j = 0; j < 7 + length; j++) {
                                p2.second[1 + j] = packet_data->second[start + j];
                            }
                            handle_packet(p2);
                        }
                        if(pos != packet_data->second.size()) { std::cout << "late end" << std::endl; break; }
                    } else {
                        handle_packet(*packet_data);
                    }
                }
            }
            
            std::vector<std::function<boost::optional<std::vector<uint8_t> >(Packet)> > persistent_patterns;
            std::map<std::vector<uint8_t>, nv08c_driver::Packet> persistent_packets;
            ros::Publisher pub;
            ros::Publisher latch_pub;
            boost::shared_ptr<Device> device;
            ros::Timer heartbeat_timer;
            bool running;
            boost::thread polling_thread_inst;
    };
    
    PLUGINLIB_DECLARE_CLASS(nv08c_driver, nodelet, nv08c_driver::Nodelet, nodelet::Nodelet);
}
