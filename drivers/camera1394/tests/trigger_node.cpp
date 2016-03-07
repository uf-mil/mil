/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <signal.h>
#include <dc1394/dc1394.h>
#include <ros/ros.h>

/** @file

    @brief test ROS node for software triggering of IIDC-compatible
    IEEE 1394 digital cameras.

*/

/** Segfault signal handler.
 *
 *  Sadly, libdc1394 sometimes crashes.
 */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  ROS_FATAL("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

class TriggerNode
{
public:

  TriggerNode():
    camera_(NULL)
  {}

  /** Set up device connection to first camera on the bus.
   *
   *  @returns true if successful
   *
   *  Failure cleanup is sketchy, but that is OK because the node 
   *  terminates, anyway.
   */
  bool setup(void)
  {
    dc1394_t *dev = dc1394_new();
    if (dev == NULL)
      {
        ROS_FATAL("Failed to initialize dc1394_context.");
        return false;
      }

    // list all 1394 cameras attached to this computer
    dc1394camera_list_t *cameras;
    int err = dc1394_camera_enumerate(dev, &cameras);
    if (err != DC1394_SUCCESS)
      {
        ROS_FATAL("Could not get list of cameras");
        return false;
      }
    if (cameras->num == 0)
      {
        ROS_FATAL("No cameras found");
        return false;
      }

    // attach to first camera found
    ROS_INFO_STREAM("Connecting to first camera, GUID: "
                    << std::setw(16) << std::setfill('0') << std::hex
                    << cameras->ids[0].guid);
    camera_ = dc1394_camera_new(dev, cameras->ids[0].guid);
    if (!camera_)
      {
        ROS_FATAL("Failed to initialize camera.");
        return false;
      }

    dc1394_camera_free_list(cameras);
    return true;
  }

  /** spin, triggering the device twice a second. */
  bool spin(void)
  {
    bool retval = true;
    ros::Rate hz(2.0);
    while (node_.ok())
      {
        ros::spinOnce();
        if (!trigger())
          {
            retval = false;
            break;
          }
        hz.sleep();
      }
    shutdown();
    return retval;
  }

private:

  /** shut down device connection */
  void shutdown(void)
  {
    dc1394_camera_free(camera_);
    camera_ = NULL;
  }

  /** Send software trigger to camera.
   *  @returns true if able to set software trigger.
   */
  bool trigger(void)
  {
    dc1394error_t err = dc1394_software_trigger_set_power(camera_, DC1394_ON);
    bool retval = (err == DC1394_SUCCESS);
    if (!retval)
      ROS_FATAL("camera does not provide software trigger");
    return retval;
  }

  ros::NodeHandle node_;
  dc1394camera_t *camera_;
};

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera1394_trigger_node");
  signal(SIGSEGV, &sigsegv_handler);

  TriggerNode trig;

  if (!trig.setup())                    // device connection failed?
    return 1;

  if (!trig.spin())                     // device does not support trigger?
    return 2;

  return 0;
}
