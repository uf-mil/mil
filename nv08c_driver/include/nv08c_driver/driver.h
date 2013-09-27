#ifndef DRIVER_H
#define DRIVER_H

#include <boost/asio/serial_port.hpp>
#include <boost/foreach.hpp>


namespace nv08c_driver {
    static const uint8_t DLE = 0x10;
    static const uint8_t ETX = 0x03;
    static const uint8_t CRC = 0xFF;
    
    class Device {
        private:
            const std::string port;
            const int baudrate;
            boost::asio::io_service io;
            boost::asio::serial_port p;
            
            bool read_byte(uint8_t &res) {
                try {
                    p.read_some(boost::asio::buffer(&res, sizeof(res)));
                    return true;
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on read: %s; reopening", exc.what());
                    open();
                    return false;
                }
            }
            
            void try_write(const std::vector<uint8_t> out) {
                try {
                    size_t written = 0;
                    while(written < out.size()) {
                        written += p.write_some(boost::asio::buffer(out.data() + written, out.size() - written));
                    }
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on write: %s; dropping", exc.what());
                }
                std::cout << "wrote: ";
                BOOST_FOREACH(const uint8_t &x, out) {
                    std::cout << std::hex << (int)x << " ";
                }
                std::cout << std::endl;
            }
            
            void open() {
                try {
                    p.close();
                    p.open(port);
                    p.set_option(boost::asio::serial_port::baud_rate(baudrate));
                    p.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::odd));
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on open(%s): %s; reopening after delay", port.c_str(), exc.what());
                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                }
            }
            
        public:
            Device(const std::string port, int baudrate) : port(port), baudrate(baudrate), p(io) {
                // open is called on first read() in the _polling_ thread
            }
            
            boost::optional<std::pair<uint8_t, std::vector<uint8_t> > > read_packet() {
                if(!p.is_open()) {
                    open();
                    return boost::none;
                }
                
                uint8_t firstbyte; if(!read_byte(firstbyte)) return boost::none;
                if(firstbyte != DLE) {
                    std::cout << "firstbyte wasn't DLE" << std::endl;
                    return boost::none;
                }
                
                uint8_t id; if(!read_byte(id)) return boost::none;
                if(id == ETX) {
                    std::cout << "id was ETX" << std::endl;
                    return boost::none;
                }
                if(id == DLE) {
                    std::cout << "id was DLE" << std::endl;
                    return boost::none;
                }
                if(id == CRC) {
                    std::cout << "id was CRC" << std::endl;
                    return boost::none;
                }
                
                std::vector<uint8_t> data;
                while(true) {
                    uint8_t b; if(!read_byte(b)) return boost::none;
                    if(b == DLE) {
                        uint8_t b2; if(!read_byte(b2)) return boost::none;
                        if(b2 == DLE) {
                            data.push_back(DLE);
                        } else if(b2 == ETX) {
                            break;
                        } else {
                            std::cout << "DLE followed by " << (int)b2 << "!" << std::endl;
                            return boost::none;
                        }
                    } else {
                        data.push_back(b);
                    }
                }
                
                return make_pair(id, data);
            }
            
            void write_packet(uint8_t const id, std::vector<uint8_t> const msg) {
                assert(id != DLE && id != ETX && id != CRC);
                
                std::vector<uint8_t> res;
                res.push_back(DLE);
                res.push_back(id);
                BOOST_FOREACH(const uint8_t &byte, msg) {
                    if(byte == DLE) {
                        res.push_back(DLE);
                        res.push_back(DLE);
                    } else {
                        res.push_back(byte);
                    }
                }
                res.push_back(DLE);
                res.push_back(ETX);
                
                try_write(res);
            }
            
            void send_heartbeat() {
                { uint8_t d[] = {}; // disable all
                  write_packet(0x0E, std::vector<uint8_t>(d, d+sizeof(d)/sizeof(d[0]))); }
                { uint8_t d[] = {0x02, 0x0A}; // set navigation rate to 10hz
                  write_packet(0xD7, std::vector<uint8_t>(d, d+sizeof(d)/sizeof(d[0]))); }
                { uint8_t d[] = {1}; // output raw data as fast as possible
                  write_packet(0xF4, std::vector<uint8_t>(d, d+sizeof(d)/sizeof(d[0]))); }
                { uint8_t d[] = {1}; // PVT too
                  write_packet(0x27, std::vector<uint8_t>(d, d+sizeof(d)/sizeof(d[0]))); }
            }
            void abort() {
                p.close();
            }
    };
    
}

#endif
