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

#include <ros/ros.h>
#include "modes.h"

/** @file

    @brief libdc1394 enumerated modes implementation

    @author Jack O'Quin
 */

////////////////////////////////////////////////////////////////
// static data and functions:
////////////////////////////////////////////////////////////////
namespace Modes
{
  // driver parameter names, corresponding to DC1394 video modes
  static const std::string video_mode_names_[DC1394_VIDEO_MODE_NUM] =
    {
      "160x120_yuv444",
      "320x240_yuv422",
      "640x480_yuv411",
      "640x480_yuv422",
      "640x480_rgb8",
      "640x480_mono8",
      "640x480_mono16",
      "800x600_yuv422",
      "800x600_rgb8",
      "800x600_mono8",
      "1024x768_yuv422",
      "1024x768_rgb8",
      "1024x768_mono8",
      "800x600_mono16",
      "1024x768_mono16",
      "1280x960_yuv422",
      "1280x960_rgb8",
      "1280x960_mono8",
      "1600x1200_yuv422",
      "1600x1200_rgb8",
      "1600x1200_mono8",
      "1280x960_mono16",
      "1600x1200_mono16",
      "exif",
      "format7_mode0",
      "format7_mode1",
      "format7_mode2",
      "format7_mode3",
      "format7_mode4",
      "format7_mode5",
      "format7_mode6",
      "format7_mode7"
    };

  /** Return driver parameter name of DC1394 video_mode.
   *
   *  @param mode DC1394 video mode number
   *  @return corresponding parameter name ("" if not a valid mode)
   */
  inline const std::string videoModeName(dc1394video_mode_t mode)
  {
    if (mode >= DC1394_VIDEO_MODE_MIN
        && mode <= DC1394_VIDEO_MODE_MAX)
      return video_mode_names_[mode - DC1394_VIDEO_MODE_MIN];
    else
      return "";
  }

  /// driver parameter names, corresponding to DC1394 color codings
  static const std::string color_coding_names_[DC1394_COLOR_CODING_NUM] =
    {
      "mono8",
      "yuv411",
      "yuv422",
      "yuv444",
      "rgb8",
      "mono16",
      "rgb16",
      "mono16s",
      "rgb16s",
      "raw8",
      "raw16",
    };

  /** Return driver parameter name of DC1394 color_coding.
   *
   *  @param mode DC1394 color coding number
   *  @return corresponding parameter name ("" if not a valid mode)
   */
  inline const std::string colorCodingName(dc1394color_coding_t mode)
  {
    if (mode >= DC1394_COLOR_CODING_MIN
        && mode <= DC1394_COLOR_CODING_MAX)
      return color_coding_names_[mode - DC1394_COLOR_CODING_MIN];
    else
      return "";
  }

  ////////////////////////////////////////////////////////////////
  // public functions:
  ////////////////////////////////////////////////////////////////

  /** Get Format7 color coding.
   *
   *  @pre camera is in a Format7 video mode.
   *
   *  @param camera points to DC1394 camera struct
   *  @param video_mode currently selected Format7 video mode.
   *  @param[in,out] color_coding Config parameter for this option,
   *                      updated if the camera does not support the
   *                      requested value
   *  @return corresponding dc1394color_coding_t enum value selected
   */
  dc1394color_coding_t getColorCoding(dc1394camera_t *camera,
                                      dc1394video_mode_t video_mode,
                                      std::string &color_coding)
  {
    for (int ccode = DC1394_COLOR_CODING_MIN;
         ccode <= DC1394_COLOR_CODING_MAX;
         ++ccode)
      {
        if (color_coding_names_[ccode-DC1394_COLOR_CODING_MIN] == color_coding)
          {
            // found the requested mode
            dc1394color_codings_t ccs;
            dc1394error_t err =
              dc1394_format7_get_color_codings(camera, video_mode, &ccs);
            if (err != DC1394_SUCCESS)
              {
                ROS_FATAL("unable to get supported color codings");
                // TODO raise exception
                return (dc1394color_coding_t) 0;
              }

            // see if requested mode is available
            for (uint32_t i = 0; i < ccs.num; ++i)
              {
                if (ccs.codings[i] == ccode)
                  return (dc1394color_coding_t) ccode; // yes: success
              }

            // requested mode not available, revert to current mode of camera
            ROS_ERROR_STREAM("Color coding " << color_coding
                             << " not supported by this camera");
            dc1394color_coding_t current_mode;
            err = dc1394_format7_get_color_coding(camera, video_mode,
                                                  &current_mode);
            if (err != DC1394_SUCCESS)
              {
                ROS_FATAL("unable to get current color coding");
                // TODO raise exception
                return (dc1394color_coding_t) 0;
              }

            // TODO list available modes

            // change color_coding parameter to show current mode of camera
            color_coding = colorCodingName(current_mode);
            return current_mode;
          }
      }

    // Requested color coding does not match any known string, set to
    // "mono8" and update parameter.
    ROS_FATAL_STREAM("Unknown color_coding: " << color_coding);
    color_coding = colorCodingName(DC1394_COLOR_CODING_MONO8);
    return (dc1394color_coding_t) DC1394_COLOR_CODING_MONO8;
  }

  /** Get non-scalable frame rate.
   *
   *  @pre camera is NOT in a Format7 video mode.
   *
   *  @param camera points to DC1394 camera struct.
   *  @param video_mode currently selected non-Format7 video mode.
   *  @param[in,out] frame_rate Config parameter for this option,
   *                 updated if the camera does not support the
   *                 requested value.
   *  @return corresponding dc1394framerate_t enum value selected,
   *                 if successful; DC1394_FRAMERATE_NUM if not.
   */
  dc1394framerate_t getFrameRate(dc1394camera_t *camera,
                                 dc1394video_mode_t video_mode,
                                 double &frame_rate)
  {
    // list frame rates supported for this video mode
    dc1394framerates_t avail_rates;
    dc1394error_t err =
      dc1394_video_get_supported_framerates(camera, video_mode, &avail_rates);
    if (err != DC1394_SUCCESS)
      {
        ROS_FATAL("getFrameRate() cannot be used for Format7 modes");
        return (dc1394framerate_t) DC1394_FRAMERATE_NUM; // failure
      }

    int result = DC1394_FRAMERATE_240;
    double rate = 240.0;

    // round frame rate down to next-lower defined value
    while (result >= DC1394_FRAMERATE_MIN)
      {
        for (uint32_t i = 0; i < avail_rates.num; ++i)
          {
            if (avail_rates.framerates[i] == result
                && rate <= frame_rate)
              {
                // update configured rate to match selected value
                frame_rate = rate;
                return (dc1394framerate_t) result;
              }
          }

        // continue with next-lower possible value
        --result;
        rate = rate / 2.0;
      }

    // no valid frame rate discovered
    ROS_ERROR("requested frame_rate (%.3f) not available", frame_rate);
    return (dc1394framerate_t) DC1394_FRAMERATE_NUM; // failure
  }

  /** Get video mode.
   *
   *  @param camera points to DC1394 camera struct
   *  @param[in,out] video_mode Config parameter for this option,
   *                      updated if the camera does not support the
   *                      requested value
   *  @return corresponding dc1394video_mode_t enum value selected
   */
  dc1394video_mode_t getVideoMode(dc1394camera_t *camera,
                                  std::string &video_mode)
  {
    for (int vm = DC1394_VIDEO_MODE_MIN;
         vm <= DC1394_VIDEO_MODE_MAX;
         ++vm)
      {
        if (video_mode_names_[vm-DC1394_VIDEO_MODE_MIN] == video_mode)
          {
            // found the requested mode
            dc1394video_modes_t vmodes;
            dc1394error_t err =
              dc1394_video_get_supported_modes(camera, &vmodes);
            if (err != DC1394_SUCCESS)
              {
                ROS_FATAL("unable to get supported video modes");
                // TODO raise exception
                return (dc1394video_mode_t) 0;
              }

            // see if requested mode is available
            for (uint32_t i = 0; i < vmodes.num; ++i)
              {
                if (vmodes.modes[i] == vm)
                  return (dc1394video_mode_t) vm; // yes: success
              }

            // requested mode not available, revert to current mode of camera
            ROS_ERROR_STREAM("Video mode " << video_mode
                             << " not supported by this camera");
            dc1394video_mode_t current_mode;
            err = dc1394_video_get_mode(camera, &current_mode);
            if (err != DC1394_SUCCESS)
              {
                ROS_FATAL("unable to get current video mode");
                // TODO raise exception
                return (dc1394video_mode_t) 0;
              }

            // TODO list available modes

            // change video_mode parameter to show current mode of camera
            video_mode = videoModeName(current_mode);
            return current_mode;
          }
      }

    // request video mode does not match any known string
    ROS_FATAL_STREAM("Unknown video_mode:" << video_mode);
    ROS_BREAK();
    // TODO raise exception
    //CAM_EXCEPT(camera1394::Exception, "Unsupported video_mode");
    return (dc1394video_mode_t) 0;
  }


  /** Set non-scalable frame rate.
   *
   *  @pre camera is NOT in a Format7 video mode.
   *
   *  @param camera points to DC1394 camera struct.
   *  @param video_mode currently selected non-Format7 video mode.
   *  @param[in,out] frame_rate Config parameter for this option,
   *                 updated if the camera does not support the
   *                 requested value.
   *  @return true if successful; false if not.
   */
  bool setFrameRate(dc1394camera_t *camera,
                    dc1394video_mode_t video_mode,
                    double &frame_rate)
  {
    dc1394framerate_t rate = getFrameRate(camera, video_mode, frame_rate);
    if (DC1394_FRAMERATE_NUM == rate)
      {
        ROS_WARN("No valid frame rate");
        return false;                   // failure
      }
    if (DC1394_SUCCESS != dc1394_video_set_framerate(camera, rate))
      {
        ROS_WARN("Failed to set frame rate");
        return false;                   // failure
      }
    return true;
  }

  /** Set ISO speed.
   *
   *  @param camera points to DC1394 camera struct
   *  @param[in,out] iso_speed Config parameter for this option,
   *                 updated if the camera does not support the
   *                 requested value
   *  @return true if ISO speed set successfully, false if not.
   *
   *  @post IEEE1394b mode enabled if camera and bus support it.
   */
  bool setIsoSpeed(dc1394camera_t *camera, int &iso_speed)
  {
    // Enable IEEE1394b mode if the camera and bus support it
    bool bmode = camera->bmode_capable;
    if (bmode
        && (DC1394_SUCCESS !=
            dc1394_video_set_operation_mode(camera,
                                            DC1394_OPERATION_MODE_1394B)))
      {
        bmode = false;
        ROS_WARN("failed to set IEEE1394b mode");
      }

    // start with highest speed supported
    dc1394speed_t request = DC1394_ISO_SPEED_3200;
    int rate = 3200;
    if (!bmode)
      {
        // not IEEE1394b capable: so 400Mb/s is the limit
        request = DC1394_ISO_SPEED_400;
        rate = 400;
      }

    // round requested speed down to next-lower defined value
    while (rate > iso_speed)
      {
        if (request <= DC1394_ISO_SPEED_MIN)
          {
            // get current ISO speed of the device
            dc1394speed_t curSpeed;
            if (DC1394_SUCCESS == dc1394_video_get_iso_speed(camera, &curSpeed)
                && curSpeed <= DC1394_ISO_SPEED_MAX)
              {
                // Translate curSpeed back to an int for the parameter
                // update, works as long as any new higher speeds keep
                // doubling.
                request = curSpeed;
                rate = 100 << (curSpeed - DC1394_ISO_SPEED_MIN);
              }
            else
              {
                ROS_WARN("Unable to get ISO speed; assuming 400Mb/s");
                rate = 400;
                request = DC1394_ISO_SPEED_400;
              }
            break;
          }

        // continue with next-lower possible value
        request = (dc1394speed_t) ((int) request - 1);
        rate = rate / 2;
      }

    // update configured rate to match selected value
    iso_speed = rate;

    // set the requested speed
    if (DC1394_SUCCESS != dc1394_video_set_iso_speed(camera, request))
      {
        ROS_WARN("Failed to set iso speed");
        return false;
      }
    
    return true;
  }

} // namespace Modes
