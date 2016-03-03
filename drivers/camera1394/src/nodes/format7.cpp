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

/** @file

    @brief Camera1394 format7 implementation

 */

#include <stdint.h>
#include "yuv.h"
#include <sensor_msgs/image_encodings.h>
#include "format7.h"
#include "modes.h"


/** Start the 1394 device in Format7 mode
 *
 *  @param camera pointer to dc1394camera_t structure.
 *  @param mode currently selected Format7 video mode.
 *  @param[in,out] newconfig new configuration parameters.
 *  @return true, if successful.
 *  @post active_ true, if successful
 */
bool Format7::start(dc1394camera_t *camera,
                    dc1394video_mode_t mode,
                    Config &newconfig)
{
  active_ = false;

  // copy Format7 parameters for updateCameraInfo()
  binning_x_ = newconfig.binning_x;
  binning_y_ = newconfig.binning_y;
  roi_.x_offset = newconfig.x_offset;
  roi_.y_offset = newconfig.y_offset;
  roi_.width = newconfig.roi_width;
  roi_.height = newconfig.roi_height;

  uint32_t packet_size = newconfig.format7_packet_size;

  // Built-in libdc1394 Bayer decoding (now deprecated) is not
  // supported at all in Format7 modes.
  if (newconfig.bayer_method != "")
    {
      ROS_WARN_STREAM("Bayer method [" << newconfig.bayer_method
                      << "] not supported for Format7 modes."
                      << "  Using image_proc instead.");
      newconfig.bayer_method = "";
    }

  const char* pattern = newconfig.bayer_pattern.c_str();
  dc1394color_filter_t bayer_pattern = findBayerPattern(pattern);
  if ((bayer_pattern >= DC1394_COLOR_FILTER_MIN)
      && (bayer_pattern <= DC1394_COLOR_FILTER_MAX))
    {
      BayerPattern_ = bayer_pattern;
    }
  else
    {
      ROS_WARN_STREAM("Bayer pattern [" << newconfig.bayer_pattern << " (" 
                      << bayer_pattern << ")] is invalid.");
    }

  // scan all Format7 modes to determine the full-sensor image size,
  // from which we will calculate the binning values
  uint32_t sensor_width = 0, sensor_height = 0;

  for (int scan_mode = DC1394_VIDEO_MODE_FORMAT7_MIN;
       scan_mode <= DC1394_VIDEO_MODE_FORMAT7_MAX;
       ++scan_mode)
    {
      uint32_t scan_width, scan_height;

      // TODO: only scan modes supported by this device
      if (dc1394_format7_get_max_image_size(camera,
                                            (dc1394video_mode_t) scan_mode,
                                            &scan_width, &scan_height)
          != DC1394_SUCCESS)
        continue;

      if (scan_width > sensor_width)
        sensor_width = scan_width;

      if (scan_height > sensor_height)
        sensor_height = scan_height;
    }

  if (DC1394_SUCCESS != dc1394_format7_get_max_image_size(camera, mode,
                                                      &maxWidth_,
                                                      &maxHeight_))
    {
      ROS_ERROR("Could not get max image size");
      return false;
    }

  if (newconfig.binning_x == 0 || newconfig.binning_y == 0)
    {
      binning_x_ = sensor_width / maxWidth_;
      binning_y_ = sensor_height / maxHeight_;
    }
  else
    {
      binning_x_ = newconfig.binning_x;
      binning_y_ = newconfig.binning_y;
    }

  maxWidth_ *= binning_x_;
  maxHeight_ *= binning_y_;

  if ((roi_.x_offset | roi_.y_offset | roi_.width | roi_.height) == 0)
    {
      roi_.width = maxWidth_;
      roi_.height = maxHeight_;
    }

  uint32_t unit_w, unit_h, unit_x, unit_y;

  if (DC1394_SUCCESS != dc1394_format7_get_unit_size(camera, mode,
                                                     &unit_w, &unit_h))
    {
      ROS_ERROR("Could not get ROI size units");
      return false;
    }

  unit_w *= binning_x_;
  unit_h *= binning_y_;

  if (DC1394_SUCCESS != dc1394_format7_get_unit_position(camera, mode,
                                                         &unit_x, &unit_y))
    {
      ROS_ERROR("Could not get ROI position units");
      return false;
    }

  // some devices return zeros for the position units
  if (unit_x == 0) unit_x = 1;
  if (unit_y == 0) unit_y = 1;

  unit_x *= binning_x_;
  unit_y *= binning_y_;

  ROS_INFO_STREAM("Format7 unit size: ("
                  << unit_w << "x" << unit_h
                  << "), position: ("
                  << unit_x << "x" << unit_y
                  << ")");
  ROS_INFO_STREAM("Format7 region size: ("
                  << roi_.width << "x" << roi_.height
                  << "), offset: ("
                  << roi_.x_offset << ", " << roi_.y_offset
                  << ")");

  /* Reset ROI position to (0,0). If it was previously (x,y) and
   * the requested ROI size (w,h) results in (x,y) + (w,h) >
   * (max_w,max_h), we'll be unable to set up some valid ROIs
   */
  dc1394_format7_set_image_position(camera, mode, 0, 0);

  if ((roi_.width % unit_w) || (roi_.height % unit_h))
    {
      /// @todo Add some sensible recovery for bad Format7 size.
      ROS_ERROR("Requested image size invalid; (w,h) must be"
                " a multiple of (%d, %d)", unit_w, unit_h);
      return false;
    }

  uint32_t binned_width = roi_.width / binning_x_;
  uint32_t binned_height = roi_.height / binning_y_;

  uint32_t binned_x_offset = roi_.x_offset / binning_x_;
  uint32_t binned_y_offset = roi_.y_offset / binning_y_;

  if (DC1394_SUCCESS != dc1394_format7_set_image_size(camera, mode,
                                                      binned_width,
                                                      binned_height))
    {
      ROS_ERROR("Could not set size of ROI");
      return false;
    }

  if ((roi_.x_offset % unit_x) || (roi_.y_offset % unit_y))
    {
      ROS_ERROR("Requested image position invalid; (x,y) must"
                " be a multiple of (%d, %d)", unit_x, unit_y);
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_set_image_position(camera, mode,
                                                          binned_x_offset,
                                                          binned_y_offset))
    {
      ROS_ERROR("Could not set position of ROI");
      return false;
    }

  // Try to set requested color coding. Use current camera value if
  // requested coding is not supported by the camera.
  coding_ = Modes::getColorCoding(camera, mode,
                                  newconfig.format7_color_coding);

  if (DC1394_SUCCESS != dc1394_format7_set_color_coding(camera, mode,
                                                        coding_))
    {
      ROS_ERROR("Could not set color coding");
      return false;
    }
      
  uint32_t rec_packet_size;

  if (DC1394_SUCCESS
      != dc1394_format7_get_recommended_packet_size(camera, mode,
                                                    &rec_packet_size))
    {
      ROS_ERROR("Could not get default packet size");
      return false;
    }

  if (0 == packet_size)
    packet_size = rec_packet_size;

  uint32_t unit_bytes, max_bytes;

  if (DC1394_SUCCESS
      != dc1394_format7_get_packet_parameters(camera, mode,
                                              &unit_bytes, &max_bytes))
    {
      ROS_ERROR("Could not determine maximum and increment for packet size");
      return false;
    }

  if (packet_size % unit_bytes
      || (max_bytes > 0 && packet_size > max_bytes))
    {
      ROS_ERROR("Invalid packet size: %d. Must be a "
                "multiple of %d, at most %d [%d]",
                packet_size, unit_bytes, max_bytes, rec_packet_size);
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_set_packet_size(camera, mode,
                                                       packet_size))
    {
      ROS_ERROR("Could not set packet size");
      return false;
    }

  if (coding_ == DC1394_COLOR_CODING_RAW8
      || coding_ == DC1394_COLOR_CODING_RAW16)
    {
      dc1394color_filter_t
        color_filter = (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM;
      if (DC1394_SUCCESS != dc1394_format7_get_color_filter(camera, mode,
                                                            &color_filter))
        {
          ROS_ERROR("Could not determine color pattern");
          return false;
        }
      if (BayerPattern_ == (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM)
        {
          BayerPattern_ = color_filter;
        }
      else if ( BayerPattern_ != color_filter )
        {
          ROS_WARN_STREAM("Bayer Pattern was set to "
                          << BayerPattern_ << "but get_color_filter returned "
                          << color_filter << ". Using " << BayerPattern_);
        }
    }

  active_ = true;                       // Format7 mode is active

  return true;
}

/** stop Format7 processing */
void Format7::stop(void)
{
  active_ = false;
}

extern std::string bayer_string(dc1394color_filter_t pattern,
                                unsigned int bits);

/** Unpack Format7 data for an Image frame */
void Format7::unpackData(sensor_msgs::Image &image, uint8_t *capture_buffer)
{
  int image_size;  
  switch (coding_)
    {
    case DC1394_COLOR_CODING_MONO8:
      image.step = image.width;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::MONO8;
      image.is_bigendian = false;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_YUV411:
      image.step = image.width * 3;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyyvyy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                      reinterpret_cast<unsigned char *> (&image.data[0]),
                      image.width * image.height);
      break;
    case DC1394_COLOR_CODING_YUV422:
      image.step = image.width * 3;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyvy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                    reinterpret_cast<unsigned char *> (&image.data[0]),
                    image.width * image.height);
      break;
    case DC1394_COLOR_CODING_YUV444:
      image.step=image.width * 3;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyv2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                   reinterpret_cast<unsigned char *> (&image.data[0]),
                   image.width * image.height);
      break;
    case DC1394_COLOR_CODING_RGB8:
      image.step=image.width*3;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_MONO16:
      image.step=image.width*2;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::MONO16;
      image.is_bigendian = true;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_RGB16:
      image.step = image.width * 6;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
      image.is_bigendian = true;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_MONO16S:
      image.step = image.width * 2;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      image.is_bigendian = true;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_RGB16S:
      image.step = image.width * 6;
      image_size = image.height * image.step;
      image.encoding = sensor_msgs::image_encodings::TYPE_16SC3;
      image.is_bigendian = true;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_RAW8:
      image.step = image.width;
      image_size = image.height * image.step;
      image.encoding = bayer_string(BayerPattern_, 8);
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_COLOR_CODING_RAW16:
      image.step = image.width * 2;
      image_size = image.height * image.step;
      image.encoding = bayer_string(BayerPattern_, 16);
      image.is_bigendian = true;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    default:
      ROS_ERROR_STREAM("Driver bug: unknown Format7 color coding:"
                       << coding_);
      ROS_BREAK();
    }
}

/** check whether CameraInfo matches current Format7 image size
 *
 *  @pre active_ is true.
 *  @param cinfo CameraInfo message to check
 *  @return true if camera dimensions match calibration
 *
 *  @post fields filled in (if successful):
 *    roi (region of interest)
 *    binning_x, binning_y
 */
bool Format7::checkCameraInfo(const sensor_msgs::CameraInfo &cinfo)
{
  // see if the (full) image size matches the calibration
  if (cinfo.width == maxWidth_ && cinfo.height == maxHeight_)
    {
      return true;
    }
  // or if the ROI size matches the calibration
  else if (cinfo.width == roi_.width && cinfo.height == roi_.height)
    {
      return true;
    }
  else
    {
      ROS_WARN_STREAM_THROTTLE(30, "Calibrated image size ("
                               << cinfo.width << "x" << cinfo.height
                               << ") matches neither full Format7 size ("
                               << maxWidth_ << "x" << maxHeight_ << ")"
                               << ") nor ROI size ("
                               << roi_.width << "x" << roi_.height << ")");
      return false;
    }
}

/** set operational data fields in CameraInfo message
 *
 *  @pre active_ is true.
 *  @param cinfo CameraInfo message to update
 *  @return true if camera dimensions match calibration
 *
 *  @post fields filled in (if successful):
 *    roi (region of interest)
 *    binning_x, binning_y
 */
void Format7::setOperationalParameters(sensor_msgs::CameraInfo &cinfo)
{
  // copy the operational data determined during start()
  cinfo.binning_x = binning_x_;
  cinfo.binning_y = binning_y_;
  cinfo.roi = roi_;

  // set do_rectify depending on current calibration parameters
  cinfo.roi.do_rectify = false;

  if (cinfo.K[0] == 0.0)
    return;				// uncalibrated

  bool roiMatchesCalibration = (cinfo.width == roi_.width
				&& cinfo.height == roi_.height);

  if (cinfo.width == maxWidth_ && cinfo.height == maxHeight_)
    {
      // calibration matches full image size
      if (!roiMatchesCalibration)
	{
	  // ROI is not full image: tell image_pipeline to rectify
	  cinfo.roi.do_rectify = true;
	}
    }
  else
    {
      // calibration differs from full image
      if (!roiMatchesCalibration)
	{
	  // calibrated size is neither full image nor current ROI:
	  //   tell image_pipeline to rectify the data.
	  cinfo.roi.do_rectify = true;
	}
    }
}

/** returns the DC1394 color filter for the given bayer pattern string
 *
 *  @param bayer the string describing the pattern (e.g. bggr, rggb, ...)
 *  @return the dc1394color_filter_t corresponding to the given string
 *
 */
dc1394color_filter_t Format7::findBayerPattern(const char* bayer)
{
  // determine Bayer color encoding pattern
  // (default is different from any color filter provided by DC1394)
  dc1394color_filter_t
    pattern = (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM;

  if (0 == strcmp(bayer, "bggr"))
    {
      pattern = DC1394_COLOR_FILTER_BGGR;
    }
  else if (0 == strcmp(bayer, "grbg"))
    {
      pattern = DC1394_COLOR_FILTER_GRBG;
    }
  else if (0 == strcmp(bayer, "rggb"))
    {
      pattern = DC1394_COLOR_FILTER_RGGB;
    }
  else if (0 == strcmp(bayer, "gbrg"))
    {
      pattern = DC1394_COLOR_FILTER_GBRG;
    }
  else if (0 != strcmp(bayer, ""))
    {
      ROS_ERROR("unknown bayer pattern [%s]", bayer);
    }
  return pattern;
}

