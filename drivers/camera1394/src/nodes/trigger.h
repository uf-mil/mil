/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013 Boris Gromov, BioRobotics Lab at Korea Tech
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

#ifndef _TRIGGER_H_
#define _TRIGGER_H_

#include <dc1394/dc1394.h>

#include "camera1394/Camera1394Config.h"
typedef camera1394::Camera1394Config Config;

/** @file

    @brief libdc1394 triggering modes interface

    Functions to get or set libdc1394 triggering modes corresponding to various
    Config parameters, limiting configured values to those actually
    supported by the device.

    @author Boris Gromov

*/

class Trigger
{
private:
  /// driver parameter names, corresponding to DC1394 trigger modes
  static const std::string trigger_mode_names_[DC1394_TRIGGER_MODE_NUM];
  /// driver parameter names, corresponding to DC1394 trigger sources
  static const std::string trigger_source_names_[DC1394_TRIGGER_SOURCE_NUM];
  /// driver parameter names, corresponding to DC1394 trigger sources
  static const std::string trigger_polarity_names_[DC1394_TRIGGER_ACTIVE_NUM];

  dc1394camera_t *camera_;

  dc1394trigger_mode_t triggerMode_;
  dc1394trigger_source_t triggerSource_;
  dc1394trigger_sources_t triggerSources_;
  dc1394trigger_polarity_t triggerPolarity_;

  dc1394switch_t externalTriggerPowerState_;

  bool findTriggerMode(std::string str);
  bool findTriggerSource(std::string str);
  bool findTriggerPolarity(std::string str);
  bool checkTriggerSource(dc1394trigger_source_t source);

public:
  /** Constructor
   *
   *  @param camera address of DC1394 camera structure.
   */
  Trigger(dc1394camera_t *camera):
    camera_(camera), triggerSources_((dc1394trigger_sources_t){0}), externalTriggerPowerState_(DC1394_OFF)
  {};

  /** Return driver parameter name of DC1394 trigger_mode.
   *
   *  @param mode DC1394 trigger mode number
   *  @return corresponding parameter name ("" if not a valid mode)
   */
  inline const std::string triggerModeName(dc1394trigger_mode_t mode)
  {
    if (mode >= DC1394_TRIGGER_MODE_MIN && mode <= DC1394_TRIGGER_MODE_MAX)
      return trigger_mode_names_[mode - DC1394_TRIGGER_MODE_MIN];
    else
      return "";
  }

  /** Return driver parameter name of DC1394 trigger_source.
   *
   *  @param mode DC1394 trigger source number
   *  @return corresponding parameter name ("" if not a valid mode)
   */
  inline const std::string triggerSourceName(dc1394trigger_source_t source)
  {
    if (source >= DC1394_TRIGGER_SOURCE_MIN && source <= DC1394_TRIGGER_SOURCE_MAX)
      return trigger_source_names_[source - DC1394_TRIGGER_SOURCE_MIN];
    else
      return "";
  }

  /** Return driver parameter name of DC1394 trigger_polarity.
   *
   *  @param mode DC1394 trigger polarity
   *  @return corresponding parameter name ("" if not a valid mode)
   */
  inline const std::string triggerPolarityName(dc1394trigger_polarity_t polarity)
  {
    if (polarity >= DC1394_TRIGGER_ACTIVE_MIN && polarity <= DC1394_TRIGGER_ACTIVE_MAX)
      return trigger_polarity_names_[polarity - DC1394_TRIGGER_ACTIVE_MIN];
    else
      return "";
  }

  /** Checks whether external trigger power is ON or OFF.
   *  This method uses cached value, which is updated every
   *  time the settings are changed by user
   *
   *  @return true if external trigger power is ON; false if not
   */
  inline bool isPowered()
  {
    return (externalTriggerPowerState_ == DC1394_ON ? true : false);
  }

  bool enumSources(dc1394camera_t *camera, dc1394trigger_sources_t &sources);
  dc1394trigger_polarity_t getPolarity(dc1394camera_t *camera);
  bool setPolarity(dc1394camera_t *camera, dc1394trigger_polarity_t &polarity);
  dc1394switch_t getExternalTriggerPowerState(dc1394camera_t *camera);
  bool setExternalTriggerPowerState(dc1394camera_t *camera, dc1394switch_t &state);
  dc1394switch_t getSoftwareTriggerPowerState(dc1394camera_t *camera);
  bool setSoftwareTriggerPowerState(dc1394camera_t *camera, dc1394switch_t &state);
  dc1394trigger_mode_t getMode(dc1394camera_t *camera);
  bool setMode(dc1394camera_t *camera, dc1394trigger_mode_t &mode);
  dc1394trigger_source_t getSource(dc1394camera_t *camera);
  bool setSource(dc1394camera_t *camera, dc1394trigger_source_t &source);

  bool reconfigure(Config *newconfig);
  bool initialize(Config *newconfig);
};

#endif // _TRIGGER_H_
