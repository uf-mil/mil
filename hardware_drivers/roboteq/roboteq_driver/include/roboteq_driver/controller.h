/**
Software License Agreement (BSD)

\file      controller.h
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ROBOTEQ_CONTROLLER
#define ROBOTEQ_CONTROLLER

#include "ros/ros.h"

#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
#include <stdint.h>
#include <string>

namespace serial {
  class Serial;
}

namespace roboteq {

class Channel;

class Controller {
  friend class Channel;

private :
  const char *port_;
  int baud_;
  bool connected_;
  bool receiving_script_messages_;
  std::string version_;
  serial::Serial *serial_;
  std::stringstream tx_buffer_;
  std::vector<Channel*> channels_;

  ros::NodeHandle nh_;
  ros::Publisher pub_status_;

  void read();
  void write(std::string);

  void processStatus(std::string msg);
  void processFeedback(std::string msg);

protected:
  // These data members are the core of the synchronization strategy in this class.
  // In short, the sendWaitAck method blocks on receiving an ack, which is passed to
  // it from the read thread using the last_response_ string.
  std::string last_response_;
  boost::mutex last_response_mutex_;
  boost::condition_variable last_response_available_;
  bool haveLastResponse() { return !last_response_.empty(); }

  // These track our progress in attempting to initialize the controller.
  uint8_t start_script_attempts_;

  class EOMSend {};

  class MessageSender {
    public:
    MessageSender(std::string init, Controller* interface)
        : init_(init), interface_(interface) {}

    template<typename T>
    MessageSender& operator<<(const T val) {
      if (ss.tellp() == 0) {
        ss << init_ << val;
      } else {
        ss << ' ' << val;
      }
      return *this;
    }

    void operator<<(EOMSend)
    {
      interface_->write(ss.str());
      ss.str("");
    }

    private:
    std::string init_;
    Controller* interface_;
    std::stringstream ss;
  };

  MessageSender command;
  MessageSender query;
  MessageSender param;
  EOMSend send, sendVerify;

public :
  Controller (const char *port, int baud);
  ~Controller();

  void addChannel(Channel* channel);
  void connect();
  bool connected() { return connected_; }
  void spinOnce() { read(); }
  void flush();

  // Send commands to motor driver.
  void setEstop() { command << "EX" << send; }
  void resetEstop() { command << "MG" << send; }
  void resetDIOx(int i) { command << "D0" << i << send; }
  void setDIOx(int i) { command << "D1" << i << send; }
  void startScript() { command << "R" << send; }
  void stopScript() { command << "R" << 0 << send; }
  void setUserVariable(int var, int val) { command << "VAR" << var << val << send; }
  void setUserBool(int var, bool val) { command << "B" << var << (val ? 1 : 0) << send; }
  bool downloadScript();

  void setSerialEcho(bool serial_echo) { param << "ECHOF" << (serial_echo ? 0 : 1) << sendVerify; }
};

}

#endif
