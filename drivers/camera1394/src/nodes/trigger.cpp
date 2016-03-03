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

#include <ros/ros.h>
#include "trigger.h"

/** @file

 @brief libdc1394 triggering modes interfaces

 @author Boris Gromov
 */

// initializing constants
const std::string Trigger::trigger_mode_names_[DC1394_TRIGGER_MODE_NUM] = {"mode_0", "mode_1", "mode_2", "mode_3", "mode_4",
                                                                         "mode_5", "mode_14", "mode_15", };
const std::string Trigger::trigger_source_names_[DC1394_TRIGGER_SOURCE_NUM] = {"source_0", "source_1", "source_2",
                                                                             "source_3", "source_software", };
const std::string Trigger::trigger_polarity_names_[DC1394_TRIGGER_ACTIVE_NUM] = {"active_low", "active_high", };

bool Trigger::findTriggerMode(std::string str)
{
  if (str == "mode_0")
    triggerMode_ = DC1394_TRIGGER_MODE_0;
  else if(str == "mode_1")
    triggerMode_ = DC1394_TRIGGER_MODE_1;
  else if(str == "mode_2")
    triggerMode_ = DC1394_TRIGGER_MODE_2;
  else if(str == "mode_3")
    triggerMode_ = DC1394_TRIGGER_MODE_3;
  else if(str == "mode_4")
    triggerMode_ = DC1394_TRIGGER_MODE_4;
  else if(str == "mode_5")
    triggerMode_ = DC1394_TRIGGER_MODE_5;
  else if(str == "mode_14")
    triggerMode_ = DC1394_TRIGGER_MODE_14;
  else if(str == "mode_15")
    triggerMode_ = DC1394_TRIGGER_MODE_15;
  else
  {
    triggerMode_ = (dc1394trigger_mode_t) DC1394_TRIGGER_MODE_NUM;
    return false;
  }

  return true;
}

bool Trigger::findTriggerSource(std::string str)
{
  if (str == "source_0")
    triggerSource_ = DC1394_TRIGGER_SOURCE_0;
  else if(str == "source_1")
    triggerSource_ = DC1394_TRIGGER_SOURCE_1;
  else if(str == "source_2")
    triggerSource_ = DC1394_TRIGGER_SOURCE_2;
  else if(str == "source_3")
    triggerSource_ = DC1394_TRIGGER_SOURCE_3;
  else if(str == "source_software")
    triggerSource_ = DC1394_TRIGGER_SOURCE_SOFTWARE;
  else
  {
    triggerSource_ = (dc1394trigger_source_t) DC1394_TRIGGER_SOURCE_NUM;
    return false;
  }

  return true;
}

bool Trigger::findTriggerPolarity(std::string str)
{
  if(str == "active_low")
    triggerPolarity_ = DC1394_TRIGGER_ACTIVE_LOW;
  else if(str == "active_high")
    triggerPolarity_ = DC1394_TRIGGER_ACTIVE_HIGH;
  else
  {
    triggerPolarity_ = (dc1394trigger_polarity_t) DC1394_TRIGGER_ACTIVE_NUM;
    return false;
  }

  return true;
}

bool Trigger::checkTriggerSource(dc1394trigger_source_t source)
{
  for(size_t i = 0; i < triggerSources_.num; i++)
    if(source == triggerSources_.sources[i]) return true;

  return false;
}

/** Get supported external trigger sources.
 *
 *  @param camera points to DC1394 camera struct
 *  @return true if successful
 */
bool Trigger::enumSources(dc1394camera_t *camera, dc1394trigger_sources_t &sources)
{
  dc1394error_t err = dc1394_external_trigger_get_supported_sources(camera, &sources);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("enumTriggerSources() failed: %d", err);
    return false; // failure
  }

    std::ostringstream ss;
    if(sources.num != 0)
    {
      for(size_t i = 0; i < sources.num - 1; i++)
        ss << triggerSourceName(sources.sources[i]) << ", ";
      ss << triggerSourceName(sources.sources[sources.num - 1]);
    }
    else
    {
      ss << "none";
    }

    ROS_DEBUG_STREAM("Trigger sources: " << ss.str());
  return true;
}

/** Get external trigger polarity.
 *
 *  @param camera points to DC1394 camera struct.
 *  @return corresponding dc1394trigger_polarity_t enum value selected,
 *                 if successful; DC1394_TRIGGER_ACTIVE_NUM if not.
 */
dc1394trigger_polarity_t Trigger::getPolarity(dc1394camera_t *camera)
{
  dc1394trigger_polarity_t current_polarity;

  dc1394bool_t has_polarity;
  dc1394error_t err = dc1394_external_trigger_has_polarity(camera, &has_polarity);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("getPolarity() failed: %d", err);
    return (dc1394trigger_polarity_t) DC1394_TRIGGER_ACTIVE_NUM; // failure
  }

  if (has_polarity == DC1394_TRUE)
  {
    err = dc1394_external_trigger_get_polarity(camera, &current_polarity);
    if (err != DC1394_SUCCESS)
    {
      ROS_FATAL("getPolarity() failed: %d", err);
      return (dc1394trigger_polarity_t) DC1394_TRIGGER_ACTIVE_NUM; // failure
    }
  }
  else
  {
    ROS_ERROR("Polarity is not supported");
    return (dc1394trigger_polarity_t) DC1394_TRIGGER_ACTIVE_NUM; // failure
  }

  return current_polarity; // success
}

/** Set external trigger polarity.
 *  @param camera points to DC1394 camera struct
 *  @param[in,out] polarity Config parameter for this option,
 *                 updated if the camera does not support the
 *                 requested value
 *  @return true if polarity set successfully, false if not.
 */
bool Trigger::setPolarity(dc1394camera_t *camera, dc1394trigger_polarity_t &polarity)
{
  dc1394trigger_polarity_t current_polarity = getPolarity(camera);

  dc1394bool_t has_polarity;
  dc1394error_t err = dc1394_external_trigger_has_polarity(camera, &has_polarity);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("setPolarity() failed: %d", err);
    return false; // failure
  }

  if (has_polarity == DC1394_TRUE)
  {
    // if config not changed, then do nothing
    if(current_polarity == polarity) return true;

    err = dc1394_external_trigger_set_polarity(camera, polarity);
    if (err != DC1394_SUCCESS)
    {
      polarity = current_polarity;
      ROS_FATAL("setPolarity() failed: %d", err);
      return false; // failure
    }
    ROS_DEBUG("setPolarity(): %s", triggerPolarityName(polarity).c_str());
  }
  else
  {
    ROS_FATAL("Polarity is not supported");
    return false; // failure
  }

  return true; // success
}

/** Get external trigger power state.
 *
 *  @param camera points to DC1394 camera struct.
 *  @return DC1394_ON for external trigger; DC1394_OFF for internal trigger.
 */
dc1394switch_t Trigger::getExternalTriggerPowerState(dc1394camera_t *camera)
{
  dc1394switch_t state;

  dc1394error_t err = dc1394_external_trigger_get_power(camera, &state);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("getExternalTriggerPowerState() failed: %d", err);
    return (dc1394switch_t)-1; // failure
  }
  externalTriggerPowerState_ = state;
  return state;
}

/** Set external trigger power state.
 *  @param camera points to DC1394 camera struct
 *  @param[in,out] state Config parameter for this option,
 *                 updated if the camera does not support the
 *                 requested value
 *  @return true if set successfully, false if not.
 */
bool Trigger::setExternalTriggerPowerState(dc1394camera_t *camera, dc1394switch_t &state)
{
  dc1394switch_t current_state = getExternalTriggerPowerState(camera);

  // if config not changed, then do nothing
  if(current_state == state) return true;

  dc1394error_t err = dc1394_external_trigger_set_power(camera, state);
  if (err != DC1394_SUCCESS)
  {
    state = current_state;
    ROS_FATAL("setExternalTriggerPowerState() failed: %d", err);
    return false; // failure
  }
  externalTriggerPowerState_ = state;
  ROS_DEBUG("setExternalTriggerPowerState(): %s", (state == DC1394_ON ? "ON" : "OFF"));
  return true; // success
}

/** Get software trigger power state.
 *
 *  @param camera points to DC1394 camera struct.
 *  @return DC1394_ON if software trigger is on; DC1394_OFF if not.
 */
dc1394switch_t Trigger::getSoftwareTriggerPowerState(dc1394camera_t *camera)
{
  dc1394switch_t state;

  dc1394error_t err = dc1394_software_trigger_get_power(camera, &state);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("getSoftwareTriggerPowerState() failed: %d", err);
    return (dc1394switch_t)-1; // failure
  }
  return state;
}

/** Set software trigger power state.
 *  @param camera points to DC1394 camera struct
 *  @param[in,out] state Config parameter for this option,
 *                 updated if the camera does not support the
 *                 requested value
 *  @return true if set successfully, false if not.
 */
bool Trigger::setSoftwareTriggerPowerState(dc1394camera_t *camera, dc1394switch_t &state)
{
  dc1394switch_t current_state = getSoftwareTriggerPowerState(camera);

  // if config not changed, then do nothing
  if(current_state == state) return true;

  dc1394error_t err = dc1394_software_trigger_set_power(camera, state);
  if (err != DC1394_SUCCESS)
  {
    state = current_state;
    ROS_FATAL("setSoftwareTriggerPowerState() failed: %d", err);
    return false; // failure
  }
  ROS_DEBUG("setSoftwareTriggerPowerState(): %s", (state == DC1394_ON ? "ON" : "OFF"));
  return true; // success
}

/** Get current trigger mode.
 *
 *  @param camera points to DC1394 camera struct.
 *  @return corresponding dc1394trigger_mode_t enum value selected,
 *                 if successful; DC1394_TRIGGER_MODE_NUM if not.
 */
dc1394trigger_mode_t Trigger::getMode(dc1394camera_t *camera)
{
  dc1394trigger_mode_t mode;

  dc1394error_t err = dc1394_external_trigger_get_mode(camera, &mode);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("getTriggerMode() failed: %d", err);
    return (dc1394trigger_mode_t) DC1394_TRIGGER_MODE_NUM; // failure
  }

  return mode;
}

/** Set external trigger mode.
 *  @param camera points to DC1394 camera struct
 *  @param[in,out] mode Config parameter for this option,
 *                 updated if the camera does not support the
 *                 requested value
 *  @return true if set successfully, false if not.
 */
bool Trigger::setMode(dc1394camera_t *camera, dc1394trigger_mode_t &mode)
{
  dc1394trigger_mode_t current_mode = getMode(camera);

  // if config not changed, then do nothing
  if(current_mode == mode) return true;

  dc1394error_t err = dc1394_external_trigger_set_mode(camera, mode);
  if (err != DC1394_SUCCESS)
  {
    mode = current_mode;
    ROS_FATAL("setTriggerMode() failed: %d", err);
    return false; // failure
  }
  ROS_DEBUG("setMode(): %s", triggerModeName(mode).c_str());
  return true; // success
}

/** Get current trigger source.
 *
 *  @param camera points to DC1394 camera struct.
 *  @return corresponding dc1394trigger_source_t enum value selected,
 *                 if successful; DC1394_TRIGGER_SOURCE_NUM if not.
 */
dc1394trigger_source_t Trigger::getSource(dc1394camera_t *camera)
{
  dc1394trigger_source_t source;

  dc1394error_t err = dc1394_external_trigger_get_source(camera, &source);
  if (err != DC1394_SUCCESS)
  {
    ROS_FATAL("getTriggerSource() failed: %d", err);
    return (dc1394trigger_source_t) DC1394_TRIGGER_SOURCE_NUM; // failure
  }

  return source;
}

/** Set external trigger source.
 *  @param camera points to DC1394 camera struct
 *  @param[in,out] source Config parameter for this option,
 *                 updated if the camera does not support the
 *                 requested value
 *  @return true if set successfully, false if not.
 */
bool Trigger::setSource(dc1394camera_t *camera, dc1394trigger_source_t &source)
{
  dc1394trigger_source_t current_source = getSource(camera);

  // if config not changed, then do nothing
  if(current_source == source) return true;

  dc1394error_t err = dc1394_external_trigger_set_source(camera, source);

  if (err != DC1394_SUCCESS)
  {
    source = current_source;
    ROS_FATAL("setTriggerSource() failed: %d", err);
    return false; // failure
  }
  ROS_DEBUG("setSource(): %s", triggerSourceName(source).c_str());

  return true; // success
}

/** reconfigures triggering parameters according to config values
 *
 *  @param newconfig [in,out] configuration parameters, updated
 *         to conform with device restrictions.
 *  @return true if successful; false if not
 */
bool Trigger::reconfigure(Config *newconfig)
{
  bool is_ok = true;

  //////////////////////////////////////////////////////////////
  // set triggering modes
  //////////////////////////////////////////////////////////////
  dc1394switch_t on_off = (dc1394switch_t) newconfig->external_trigger;
  if (!Trigger::setExternalTriggerPowerState(camera_, on_off))
  {
    newconfig->external_trigger = on_off;
    ROS_ERROR("Failed to set external trigger power");
    is_ok = false;
  }

  on_off = (dc1394switch_t) newconfig->software_trigger;
  if (!Trigger::setSoftwareTriggerPowerState(camera_, on_off))
  {
    newconfig->software_trigger = on_off;
    ROS_ERROR("Failed to set software trigger power");
    is_ok = false;
  }

  if (findTriggerMode(newconfig->trigger_mode))
  {
    if (!Trigger::setMode(camera_, triggerMode_))
    {
      // Possible if driver compiled against different version of libdc1394
      ROS_ASSERT(triggerMode_ <= DC1394_TRIGGER_MODE_MAX);
      newconfig->trigger_mode = Trigger::trigger_mode_names_[triggerMode_ - DC1394_TRIGGER_MODE_MIN];
      ROS_ERROR("Failed to set trigger mode");
      is_ok = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unknown trigger mode: " << newconfig->trigger_mode);
    is_ok = false;
  }

  if (triggerSources_.num != 0)
  {
    if (findTriggerSource(newconfig->trigger_source) && checkTriggerSource(triggerSource_))
    {
      if (!Trigger::setSource(camera_, triggerSource_))
      {
        // Possible if driver compiled against different version of libdc1394
        ROS_ASSERT(triggerSource_ <= DC1394_TRIGGER_SOURCE_MAX);
        newconfig->trigger_source = Trigger::trigger_source_names_[triggerSource_ - DC1394_TRIGGER_SOURCE_MIN];
        ROS_ERROR("Failed to set trigger source");
        is_ok = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unknown trigger source: " << newconfig->trigger_source);
      is_ok = false;
    }
  }
  else
  {
    ROS_DEBUG("No triggering sources available");
  }

  if (findTriggerPolarity(newconfig->trigger_polarity))
  {
    if (!Trigger::setPolarity(camera_, triggerPolarity_))
    {
      // Possible if driver compiled against different version of libdc1394
      ROS_ASSERT(triggerPolarity_ <= DC1394_TRIGGER_ACTIVE_MAX);
      newconfig->trigger_polarity = Trigger::trigger_polarity_names_[triggerPolarity_ - DC1394_TRIGGER_ACTIVE_MIN];
      ROS_ERROR("Failed to set trigger polarity");
      is_ok = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unknown trigger polarity: " << newconfig->trigger_polarity);
    is_ok = false;
  }

  return is_ok;
}

/** enumerates trigger sources and configures triggering parameters
 *  according to config values
 *
 *  @param newconfig [in,out] configuration parameters, updated
 *         to conform with device restrictions.
 *  @return true if successful; false if not
 */
bool Trigger::initialize(Config *newconfig)
{
  ROS_INFO("[%016lx] has trigger support", camera_->guid);

  // Enumerate trigger sources
  if (!Trigger::enumSources(camera_, triggerSources_))
  {
    ROS_ERROR("Failed to enumerate trigger sources");
    return false;
  }

  // Update externalTriggerPowerState_ variable with current value
  Trigger::getExternalTriggerPowerState(camera_);

  // configure trigger features
  return reconfigure(newconfig);
}
