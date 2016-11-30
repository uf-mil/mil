#ifndef _IVXONIZWBGYIXQWV_
#define _IVXONIZWBGYIXQWV_

#include <random>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>

#include <uf_subbus_protocol/protocol.h>
#include <uf_subbus_protocol/sha256.h>
#include <arm_bootloader/protocol.h>

namespace arm_bootloader {


bool write_data(boost::asio::serial_port &sp, uint8_t const *begin, uint8_t const *end) {
  try {
    uint8_t const *pos = begin;
    while(pos != end) {
      pos += sp.write_some(boost::asio::buffer(pos, end - pos));
    }
    return true;
  } catch(const std::exception &exc) {
    //ROS_ERROR("error on write: %s; dropping", exc.what());
    return false;
  }
}

class SerialPortSink : public uf_subbus_protocol::ISink {
  std::string serial_port_filename;
  boost::asio::serial_port &sp;
  bool had_error;
public:
  SerialPortSink(std::string const &serial_port_filename, boost::asio::serial_port &sp) :
    serial_port_filename(serial_port_filename), sp(sp), had_error(false) {
  }
  void handleStart() {
  }
  void handleByte(uint8_t byte) {
    while(!write_data(sp, &byte, &byte + 1)) {
      had_error = true;
      sp.close();
      while(true) {
        try {
          sp.open(serial_port_filename);
        } catch(const std::exception &exc) {
          //ROS_ERROR("error on open: %s; retrying", exc.what());	//Silencing repetitive error messages
          usleep(500000);
          continue;
        }
        break;
      }
    }
  }
  void handleEnd() {
  }
  
  bool getHadError() {
    return had_error;
  }
};


template<typename Response>
class Reader {
  boost::asio::serial_port &sp;
  double timeout;
  uint8_t buf;
  boost::optional<Response> result;
  boost::asio::deadline_timer timer;
  
  void read_complete(const boost::system::error_code& error,
                      size_t bytes_transferred, boost::function<void(uint8_t)> callback) {   

      bool read_error = error || bytes_transferred == 0;
      
      if(!read_error) {
        callback(buf);
        if(result) return;
        // Asynchronously read 1 character.
        boost::asio::async_read(sp, boost::asio::buffer(&buf, 1),
                boost::bind(&Reader::read_complete,
                        this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred,
                        callback));
      }
  }
  void time_out(const boost::system::error_code& error) {
    if(error) return; // was cancelled
    sp.cancel();
  }
  void handleResult(Response const &response, ID id_desired) {
    if(response.id != id_desired) return;
    result = response;
    // Read has finished, so cancel the
    // timer.
    timer.cancel();
    sp.cancel();
  }

public:
  Reader(boost::asio::serial_port &sp,
         double timeout) : sp(sp), timeout(timeout), timer(sp.get_io_service()) {
  }
  
  boost::optional<Response> read(ID id) {
    boost::function<void(const Response &)> tmp = boost::bind(&Reader::handleResult, this, _1, id);
    
    typedef uf_subbus_protocol::SimpleReceiver
      <Response, boost::function<void(const Response &)> > Receiver;
    
    Receiver receiver(tmp);
    
    result = boost::none;
    
    // After a timeout & cancel it seems we need
    // to do a reset for subsequent reads to work.
    sp.get_io_service().reset();

    boost::function<void(uint8_t)> tmp2 = boost::bind(&Receiver::handleRawByte, receiver, _1);
    // Asynchronously read 1 character.
    boost::asio::async_read(sp, boost::asio::buffer(&buf, 1),
            boost::bind(&Reader::read_complete,
                    this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred,
                    tmp2));

    // Setup a deadline time to implement our timeout.
    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait(boost::bind(&Reader::time_out,
                            this, boost::asio::placeholders::error));

    // This will block until a character is read
    // or until the it is cancelled.
    sp.get_io_service().run();

    return result;
  }
};


bool attempt_bootload(std::string serial_port_filename,
                      boost::asio::serial_port &sp,
                      Dest dest,
                      unsigned char const *firmware_bin,
                      uint32_t firmware_bin_len) {
  boost::asio::io_service io;
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 65535);
  
  Reader<Response> reader(sp, 1000);
  
  SerialPortSink sps(serial_port_filename, sp);
  uf_subbus_protocol::SimpleSender<Command, uf_subbus_protocol::ISink> sender(sps);
  
unsure_if_talking_to_bootloader:
  {
    Command cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.dest = dest;
    cmd.id = dis(gen);
    cmd.command = CommandID::GetStatus;
    sender.write_object(cmd);
    
    boost::optional<Response> resp = reader.read(cmd.id);
    if(resp && resp->resp.GetStatus.magic == GetStatusResponse::MAGIC_VALUE) {
      std::cout << "talking to bootloader!" << std::endl;
    } else {
      if(!resp) {
        std::cout << "timeout while querying bootloader!" << std::endl;
      } else {
        std::cout << "not talking to bootloader! " << resp->resp.GetStatus.magic << std::endl;
      }
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::Reset;
      sender.write_object(cmd);
      
      usleep(300000);
      
      goto unsure_if_talking_to_bootloader;
    }
  }

unsure_if_program_correct:
  bool hash_matched;
  {
    Command cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.dest = dest;
    cmd.id = dis(gen);
    cmd.command = CommandID::GetProgramHash;
    cmd.args.GetProgramHash.length = firmware_bin_len;
    sender.write_object(cmd);
    
    boost::optional<Response> resp = reader.read(cmd.id);
    if(!resp) {
      std::cout << "getting program hash timed out!" << std::endl;
      goto unsure_if_talking_to_bootloader;
    }
    if(!resp->resp.GetProgramHash.error_number) {
      std::cout << "hash operation succeeded!" << std::endl;
    } else {
      std::cout << "hash operation failed! (error_number = " << (int)resp->resp.GetProgramHash.error_number << ")" << std::endl;
      return false;
    }
    
    uf_subbus_protocol::sha256_state md; uf_subbus_protocol::sha256_init(md);
    uf_subbus_protocol::sha256_process(md, firmware_bin, firmware_bin_len);
    uint8_t hash[32]; uf_subbus_protocol::sha256_done(md, hash);
    
    hash_matched = true;
    for(int i = 0; i < 32; i++) {
      if(resp->resp.GetProgramHash.hash[i] != hash[i]) {
        hash_matched = false;
        break;
      }
    }
  }
  
  if(!hash_matched) {
    std::cout << "hash doesn't match!" << std::endl;
    
    {
      std::cout << "Erasing..." << std::endl;
      
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::Erase;
      cmd.args.Erase.min_length = firmware_bin_len;
      sender.write_object(cmd);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "flashing page timed out!" << std::endl;
        goto unsure_if_talking_to_bootloader;
      }
      if(resp->resp.Flash.error_number) {
        std::cout << "    ...failed! (error_number = " << (int)resp->resp.Flash.error_number << ")" << std::endl;
        return false;
      }
      
      std::cout << "    ...succeeded!" << std::endl;
    }
    
    for(uint32_t start_offset = 0; start_offset < firmware_bin_len; start_offset += FlashCommand::MAX_LENGTH) {
      uint32_t length = firmware_bin_len - start_offset;
      if(length > FlashCommand::MAX_LENGTH) length = FlashCommand::MAX_LENGTH;
      std::cout << "Flashing is " << (int)100.*start_offset/firmware_bin_len << "% done." << std::endl;
      
      Command cmd; memset(&cmd, 0, sizeof(cmd));
      cmd.dest = dest;
      cmd.id = dis(gen);
      cmd.command = CommandID::Flash;
      cmd.args.Flash.start_offset = start_offset;
      cmd.args.Flash.length = length;
      memcpy(cmd.args.Flash.data, firmware_bin + start_offset, length);
      sender.write_object(cmd);
      
      boost::optional<Response> resp = reader.read(cmd.id);
      if(!resp) {
        std::cout << "flashing page timed out!" << std::endl;
        goto unsure_if_talking_to_bootloader;
      }
      if(resp->resp.Flash.error_number) {
        std::cout << "    ...failed! (error_number = " << (int)resp->resp.Flash.error_number << ")" << std::endl;
        return false;
      }
      
      std::cout << "    ...succeeded!" << std::endl;
    }
    
    std::cout << "finished programming!" << std::endl;
    
    goto unsure_if_program_correct;
  }
  
  std::cout << "hash matches!" << std::endl;
  
  {
    Command cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.dest = dest;
    cmd.id = dis(gen);
    cmd.command = CommandID::RunProgram;
    sender.write_object(cmd);
    
    boost::optional<Response> resp = reader.read(cmd.id);
    if(!resp) {
      std::cout << "starting program timed out!" << std::endl;
      goto unsure_if_talking_to_bootloader;
    }
  }
  
  std::cout << "started talking to xbee" << std::endl;
  
  return true;
}


}

#endif
