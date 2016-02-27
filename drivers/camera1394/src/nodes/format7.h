/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Ken Tossell, Jack O'Quin
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

#ifndef _FORMAT7_H_
#define _FORMAT7_H_

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <dc1394/dc1394.h>

#include "camera1394/Camera1394Config.h"
typedef camera1394::Camera1394Config Config;

/** @file

    @brief Camera1394 Format7 interface

    @author Ken Tossell, Jack O'Quin
 */

/*** @brief Camera1394 Format7 class
 *
 *   Sets CameraInfo Format7 data from Config updates.  Tracks values
 *   and ranges, modifying configured values to those supported by the
 *   device.
 *
 */
class Format7
{
public:

  Format7():
    active_(false),
    coding_(DC1394_COLOR_CODING_MONO8),
    maxWidth_(0),
    maxHeight_(0),
    binning_x_(0),
    binning_y_(0),
    BayerPattern_((dc1394color_filter_t) DC1394_COLOR_FILTER_NUM)
  {};
  ~Format7() {};

  /** Format7 mode currently started */
  bool active(void)
  {
    return active_;
  }
  bool start(dc1394camera_t *camera, dc1394video_mode_t mode,
             Config &newconfig);
  void stop(void);
  void unpackData(sensor_msgs::Image &image, uint8_t *capture_buffer);
  bool checkCameraInfo(const sensor_msgs::CameraInfo &cinfo);
  void setOperationalParameters(sensor_msgs::CameraInfo &cinfo);

private:
  dc1394color_filter_t findBayerPattern(const char* bayer);

  bool active_;
  dc1394color_coding_t coding_;
  uint32_t maxWidth_;
  uint32_t maxHeight_;

  /** currently configured region of interest */
  sensor_msgs::RegionOfInterest roi_;

  /** current Format7 video mode binning */
  uint32_t binning_x_;
  uint32_t binning_y_;

  /** order of pixels in raw image format */
  dc1394color_filter_t BayerPattern_;
};

#endif // _FORMAT7_H_
