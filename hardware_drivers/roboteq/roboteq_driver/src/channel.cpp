/**
Software License Agreement (BSD)

\file      channel.cpp
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

#include "roboteq_driver/channel.h"
#include "roboteq_driver/controller.h"

#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"
#include "roboteq_msgs/Command.h"


namespace roboteq {

Channel::Channel(int channel_num, std::string ns, Controller* controller) :
  channel_num_(channel_num), nh_(ns), controller_(controller), max_rpm_(3500),
  last_mode_(255)
{
  sub_cmd_ = nh_.subscribe("cmd", 1, &Channel::cmdCallback, this);
  pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);

  // Don't start this timer until we've received the first motion command, otherwise it
  // can interfere with code download on device startup.
  timeout_timer_ = nh_.createTimer(ros::Duration(0.1), &Channel::timeoutCallback, this);
  timeout_timer_.stop();
}

void Channel::cmdCallback(const roboteq_msgs::Command& command)
{
  // Reset command timeout.
  timeout_timer_.stop();
  timeout_timer_.start();

  // Update mode of motor driver. We send this on each command for redundancy against a
  // lost message, and the MBS script keeps track of changes and updates the control
  // constants accordingly.
  controller_->command << "VAR" << channel_num_ << static_cast<int>(command.mode) << controller_->send;

  if (command.mode == roboteq_msgs::Command::MODE_VELOCITY)
  {
    // Get a -1000 .. 1000 command as a proportion of the maximum RPM.
    int roboteq_velocity = to_rpm(command.setpoint) / max_rpm_ * 1000.0;
    ROS_DEBUG_STREAM("Commanding " << roboteq_velocity << " velocity to motor driver.");

    // Write mode and command to the motor driver.
    controller_->command << "G" << channel_num_ << roboteq_velocity << controller_->send;
  }
  else if (command.mode == roboteq_msgs::Command::MODE_POSITION)
  {
    // Convert the commanded position in rads to encoder ticks.
    int roboteq_position = to_encoder_ticks(command.setpoint);
    ROS_DEBUG_STREAM("Commanding " << roboteq_position << " position to motor driver.");

    // Write command to the motor driver.
    controller_->command << "P" << channel_num_ << roboteq_position << controller_->send;
  }
  else
  {
    ROS_WARN_STREAM("Command received with unknown mode number, dropping.");
  }

  controller_->flush();
  last_mode_ = command.mode;
}

void Channel::timeoutCallback(const ros::TimerEvent&)
{
  // Sends stop command repeatedly at 10Hz when not being otherwise commanded. Sending
  // repeatedly is a hedge against a lost serial message.
  ROS_DEBUG("Commanding motor to stop due to user command timeout.");
  controller_->command << "VAR" << channel_num_ << static_cast<int>(roboteq_msgs::Command::MODE_STOPPED) << controller_->send;
  controller_->flush();
}

void Channel::feedbackCallback(std::vector<std::string> fields)
{
  roboteq_msgs::Feedback msg;
  msg.header.stamp = last_feedback_time_ = ros::Time::now();

  // Scale factors as outlined in the relevant portions of the user manual, please
  // see mbs/script.mbs for URL and specific page references.
  try
  {
    msg.motor_current = boost::lexical_cast<float>(fields[2]) / 10;
    msg.commanded_velocity = from_rpm(boost::lexical_cast<double>(fields[3]));
    msg.motor_power = boost::lexical_cast<float>(fields[4]) / 1000.0;
    msg.measured_velocity = from_rpm(boost::lexical_cast<double>(fields[5]));
    msg.measured_position = from_encoder_ticks(boost::lexical_cast<double>(fields[6]));
    msg.supply_voltage = boost::lexical_cast<float>(fields[7]) / 10.0;
    msg.supply_current = boost::lexical_cast<float>(fields[8]) / 10.0;
    msg.motor_temperature = boost::lexical_cast<int>(fields[9]) * 0.020153 - 4.1754;
    msg.channel_temperature = boost::lexical_cast<int>(fields[10]);
  }
  catch (std::bad_cast& e)
  {
    ROS_WARN("Failure parsing feedback data. Dropping message.");
    return;
  }
  pub_feedback_.publish(msg);
}

}
