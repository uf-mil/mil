/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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

#ifndef _MODES_H_
#define _MODES_H_

#include <dc1394/dc1394.h>

/** @file

    @brief libdc1394 enumerated modes interface

    Functions to get or set libdc1394 modes corresponding to various
    Config parameters, limiting configured values to those actually
    supported by the device.

    @author Jack O'Quin

*/

namespace Modes
{
  dc1394color_coding_t getColorCoding(dc1394camera_t *camera,
                                      dc1394video_mode_t video_mode,
                                      std::string &color_coding);
  dc1394framerate_t getFrameRate(dc1394camera_t *camera,
                                 dc1394video_mode_t video_mode,
                                 double &frame_rate);
  dc1394video_mode_t getVideoMode(dc1394camera_t *camera,
                                  std::string &video_mode);

  bool setFrameRate(dc1394camera_t *camera,
                    dc1394video_mode_t video_mode,
                    double &frame_rate);
  bool setIsoSpeed(dc1394camera_t *camera, int &iso_speed);
}

#endif // _MODES_H_
