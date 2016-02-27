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

#include <cmath>
#include "features.h"
#include "trigger.h"

/** @file

    @brief Camera1394 features implementation

    @author Jack O'Quin
 */


////////////////////////////////////////////////////////////////
// static data and functions:
////////////////////////////////////////////////////////////////
namespace
{
  // driver feature parameter names, corresponding to DC1394 modes
  // (not all currently supported by the driver)
  static const char *feature_names_[DC1394_FEATURE_NUM] =
    {
      "brightness",
      "exposure",
      "sharpness",
      "whitebalance",
      "hue",
      "saturation",
      "gamma",
      "shutter",
      "gain",
      "iris",
      "focus",
      "temperature",
      "trigger",
      "trigger_delay",
      "white_shading",
      "frame_rate",
      "zoom",
      "pan",
      "tilt",
      "optical_filter",
      "capture_size",
      "capture_quality"
    };

  /** Return driver parameter name of DC1394 feature for logging.
   *
   *  @param feature feature number
   *  @return feature name character string
   */
  inline const char *featureName(dc1394feature_t feature)
  {
    if (feature >= DC1394_FEATURE_MIN && feature <= DC1394_FEATURE_MAX)
      return feature_names_[feature - DC1394_FEATURE_MIN];
    else
      return "(unknown)";
  }

  // driver mode parameter names, corresponding to DC1394 modes
  static const char *mode_names_[DC1394_FEATURE_MODE_NUM] =
    {
      "Manual",
      "Auto",
      "OnePush",
    };

  /** Return driver parameter name of DC1394 feature mode for logging.
   *
   *  @param mode feature mode number
   *  @return mode name character string
   */
  inline const char *modeName(dc1394feature_mode_t mode)
  {
    if (mode >= DC1394_FEATURE_MODE_MIN && mode <= DC1394_FEATURE_MODE_MAX)
      return mode_names_[mode - DC1394_FEATURE_MODE_MIN];
    else
      return "(unknown)";
  }
}

////////////////////////////////////////////////////////////////
// public methods:
////////////////////////////////////////////////////////////////

/** Constructor
 *
 *  @param camera address of DC1394 camera structure.
 */
Features::Features(dc1394camera_t *camera):
  camera_(camera)
{
  trigger_.reset(new Trigger(camera));
}

/** Query and set all features for newly opened (or reopened) device.
 *
 *  @param newconfig [in,out] configuration parameters, updated
 *         to conform with device restrictions.
 *  @return true if successful
 *
 *  @post feature_set_ initialized, if successful
 *  @post oldconfig_ settings available, if successful
 */
bool Features::initialize(Config *newconfig)
{
  bool retval = true;

  // query all features for this device
  if (DC1394_SUCCESS != dc1394_feature_get_all(camera_, &feature_set_))
    {
      ROS_ERROR("could not get camera feature information");
      return false;
    }

  // validate and set configured value of each supported feature
  configure(DC1394_FEATURE_BRIGHTNESS,
            &newconfig->auto_brightness, &newconfig->brightness);
  configure(DC1394_FEATURE_EXPOSURE,
            &newconfig->auto_exposure, &newconfig->exposure);
  configure(DC1394_FEATURE_FOCUS,
            &newconfig->auto_focus, &newconfig->focus);
  configure(DC1394_FEATURE_GAIN,
            &newconfig->auto_gain, &newconfig->gain);
  configure(DC1394_FEATURE_GAMMA,
            &newconfig->auto_gamma, &newconfig->gamma);
  configure(DC1394_FEATURE_HUE,
            &newconfig->auto_hue, &newconfig->hue);
  configure(DC1394_FEATURE_IRIS,
            &newconfig->auto_iris, &newconfig->iris);
  configure(DC1394_FEATURE_PAN,
            &newconfig->auto_pan, &newconfig->pan);
  configure(DC1394_FEATURE_SATURATION,
            &newconfig->auto_saturation, &newconfig->saturation);
  configure(DC1394_FEATURE_SHARPNESS,
            &newconfig->auto_sharpness, &newconfig->sharpness);
  configure(DC1394_FEATURE_SHUTTER,
            &newconfig->auto_shutter, &newconfig->shutter);
  configure(DC1394_FEATURE_TRIGGER,
            &newconfig->auto_trigger, &newconfig->trigger);
  configure(DC1394_FEATURE_WHITE_BALANCE, &newconfig->auto_white_balance,
            &newconfig->white_balance_BU, &newconfig->white_balance_RV);
  configure(DC1394_FEATURE_ZOOM,
            &newconfig->auto_zoom, &newconfig->zoom);

  // set up trigger class, if supported by this camera
  if (hasTrigger())
    retval = trigger_->initialize(newconfig);

  // save configured values
  oldconfig_ = *newconfig;
  return retval;
}

/** Reconfigure features for already open device.
 *
 *  For each supported feature that has changed, update the device.
 *
 *  @pre feature_set_ initialized
 *  @pre oldconfig_ has previous settings
 *
 *  @param newconfig [in,out] configuration parameters, may be updated
 *         to conform with device restrictions.
 *
 *  @post oldconfig_ settings updated
 */
void Features::reconfigure(Config *newconfig)
{
  updateIfChanged(DC1394_FEATURE_BRIGHTNESS,
                  oldconfig_.auto_brightness, &newconfig->auto_brightness,
                  oldconfig_.brightness, &newconfig->brightness);
  updateIfChanged(DC1394_FEATURE_EXPOSURE,
                  oldconfig_.auto_exposure, &newconfig->auto_exposure,
                  oldconfig_.exposure, &newconfig->exposure);
  updateIfChanged(DC1394_FEATURE_FOCUS,
                  oldconfig_.auto_focus, &newconfig->auto_focus,
 		  oldconfig_.focus, &newconfig->focus);
  updateIfChanged(DC1394_FEATURE_GAIN,
                  oldconfig_.auto_gain, &newconfig->auto_gain,
                  oldconfig_.gain, &newconfig->gain);
  updateIfChanged(DC1394_FEATURE_GAMMA,
                  oldconfig_.auto_gamma, &newconfig->auto_gamma,
                  oldconfig_.gamma, &newconfig->gamma);
  updateIfChanged(DC1394_FEATURE_HUE,
                  oldconfig_.auto_hue, &newconfig->auto_hue,
                  oldconfig_.hue, &newconfig->hue);
  updateIfChanged(DC1394_FEATURE_IRIS,
                  oldconfig_.auto_iris, &newconfig->auto_iris,
                  oldconfig_.iris, &newconfig->iris);
  updateIfChanged(DC1394_FEATURE_PAN,
                  oldconfig_.auto_pan, &newconfig->auto_pan,
 		  oldconfig_.pan, &newconfig->pan);
  updateIfChanged(DC1394_FEATURE_SATURATION,
                  oldconfig_.auto_saturation, &newconfig->auto_saturation,
                  oldconfig_.saturation, &newconfig->saturation);
  updateIfChanged(DC1394_FEATURE_SHARPNESS,
                  oldconfig_.auto_sharpness, &newconfig->auto_sharpness,
                  oldconfig_.sharpness, &newconfig->sharpness);
  updateIfChanged(DC1394_FEATURE_SHUTTER,
                  oldconfig_.auto_shutter, &newconfig->auto_shutter,
                  oldconfig_.shutter, &newconfig->shutter);
  updateIfChanged(DC1394_FEATURE_TRIGGER,
                  oldconfig_.auto_trigger, &newconfig->auto_trigger,
                  oldconfig_.trigger, &newconfig->trigger);
  // White balance has two component parameters: Blue/U and Red/V.
  updateIfChanged(DC1394_FEATURE_WHITE_BALANCE,
                  oldconfig_.auto_white_balance,
                  &newconfig->auto_white_balance,
                  oldconfig_.white_balance_BU, &newconfig->white_balance_BU,
                  oldconfig_.white_balance_RV, &newconfig->white_balance_RV);
  updateIfChanged(DC1394_FEATURE_ZOOM,
                  oldconfig_.auto_zoom, &newconfig->auto_zoom,
 		  oldconfig_.zoom, &newconfig->zoom);

  // reconfigure trigger class, if supported by this camera
  if (hasTrigger())
    trigger_->reconfigure(newconfig);

  // save modified values
  oldconfig_ = *newconfig;
}

////////////////////////////////////////////////////////////////
// private methods:
////////////////////////////////////////////////////////////////

/** Configure a feature for the currently open device.
 *
 *  @pre feature_set_ initialized
 *
 *  @param feature desired feature number
 *  @param control [in,out] pointer to control parameter (may change)
 *  @param value [in,out] pointer to requested parameter value (may
 *               change depending on device restrictions)
 *  @param value2 [in,out] optional pointer to second parameter value
 *               for white balance (may change depending on device
 *               restrictions).  No second value if NULL pointer.
 *
 *  The parameter values are represented as double despite the fact
 *  that most features on most cameras only allow unsigned 12-bit
 *  values.  The exception is the rare feature that supports
 *  "absolute" values in 32-bit IEEE float format.  Double can
 *  represent all possible option values accurately.
 */
void Features::configure(dc1394feature_t feature, int *control,
                         double *value, double *value2)
{
  // device-relevant information for this feature
  dc1394feature_info_t *finfo =
    &feature_set_.feature[feature - DC1394_FEATURE_MIN];

  if (!finfo->available)                // feature not available?
    {
      *control = camera1394::Camera1394_None;
      return;
    }

  switch (*control)
    {
    case camera1394::Camera1394_Off:
      setPower(finfo, DC1394_OFF);
      break;

    case camera1394::Camera1394_Query:
      getValues(finfo, value, value2);
      break;

    case camera1394::Camera1394_Auto:
      if (!setMode(finfo, DC1394_FEATURE_MODE_AUTO))
        {
          setPower(finfo, DC1394_OFF);
        }
      break;

    case camera1394::Camera1394_Manual:
      if (!setMode(finfo, DC1394_FEATURE_MODE_MANUAL))
        {
          setPower(finfo, DC1394_OFF);
          break;
        }

      // TODO: break this into some internal methods
      if (finfo->absolute_capable && finfo->abs_control)
        {
          // supports reading and setting float value
          float fmin, fmax;
          if (DC1394_SUCCESS ==
              dc1394_feature_get_absolute_boundaries(camera_, feature,
                                                     &fmin, &fmax))
            {
              // clamp *value between minimum and maximum
              if (*value < fmin)
                *value = (double) fmin;
              else if (*value > fmax)
                *value = (double) fmax;
            }
          else
            {
              ROS_WARN_STREAM("failed to get feature "
                              << featureName(feature) << " boundaries ");
            }

          // @todo handle absolute White Balance values
          float fval = *value;
          if (DC1394_SUCCESS !=
              dc1394_feature_set_absolute_value(camera_, feature, fval))
            {
              ROS_WARN_STREAM("failed to set feature "
                              << featureName(feature) << " to " << fval);
            }
        }
      else // no float representation
        {
          // round requested value to nearest integer
          *value = rint(*value);

          // clamp *value between minimum and maximum
          if (*value < finfo->min)
            *value = (double) finfo->min;
          else if (*value > finfo->max)
            *value = (double) finfo->max;

          dc1394error_t rc;
          uint32_t ival = (uint32_t) *value;

          // handle White Balance, which has two components
          if (feature == DC1394_FEATURE_WHITE_BALANCE)
            {
              *value2 = rint(*value2);

              // clamp *value2 between same minimum and maximum
              if (*value2 < finfo->min)
                *value2 = (double) finfo->min;
              else if (*value2 > finfo->max)
                *value2 = (double) finfo->max;

              uint32_t ival2 = (uint32_t) *value2;
              rc = dc1394_feature_whitebalance_set_value(camera_, ival, ival2);
            }
          else
            {
              // other features only have one component
              rc = dc1394_feature_set_value(camera_, feature, ival);
            }
          if (rc != DC1394_SUCCESS)
            {
              ROS_WARN_STREAM("failed to set feature "
                              << featureName(feature) << " to " << ival);
            }
        }
      break;

    case camera1394::Camera1394_OnePush:
      // Try to set OnePush mode
      setMode(finfo, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);

      // Now turn the control off, so camera does not continue adjusting
      setPower(finfo, DC1394_OFF);
      break;

    case camera1394::Camera1394_None:
      // Invalid user input, because this feature actually does exist.
      ROS_INFO_STREAM("feature " << featureName(feature)
                      << " exists, cannot set to None");
      break;

    default:
      ROS_WARN_STREAM("unknown state (" << *control
                      <<") for feature " << featureName(feature));
    }

  // return actual state reported by the device
  *control = getState(finfo);
  ROS_DEBUG_STREAM("feature " << featureName(feature)
                   << " now in state " << *control);
}

/** Get current state of a feature from the camera.
 *
 *  @pre feature_set_ initialized for this camera
 *
 *  @param finfo pointer to information for this feature
 *  @return current state ID
 */
Features::state_t Features::getState(dc1394feature_info_t *finfo)
{
  dc1394feature_t feature = finfo->id;
  dc1394error_t rc;

  if (!finfo->available)
    {
      // not available: nothing more to do
      return camera1394::Camera1394_None;
    }

  if (finfo->on_off_capable)
    {
      // get On/Off state
      dc1394switch_t pwr;
      rc = dc1394_feature_get_power(camera_, feature, &pwr);
      if (rc != DC1394_SUCCESS)
        {
          ROS_WARN_STREAM("failed to get feature " << featureName(feature)
                          << " Power setting ");
        }
      else if (pwr == DC1394_OFF)
        {
          // Off overrides mode settings
          return camera1394::Camera1394_Off;
        }
    }

  // not off, so get mode
  dc1394feature_mode_t mode;
  rc = dc1394_feature_get_mode(camera_, feature, &mode);
  if (rc != DC1394_SUCCESS)
    {
      ROS_WARN_STREAM("failed to get current mode of feature "
                      << featureName(feature));
      // treat unavailable mode as Off
      return camera1394::Camera1394_Off;
    }

  switch (mode)
    {
    case DC1394_FEATURE_MODE_MANUAL:
      return camera1394::Camera1394_Manual;
    case DC1394_FEATURE_MODE_AUTO:
      return camera1394::Camera1394_Auto;
    case DC1394_FEATURE_MODE_ONE_PUSH_AUTO:
      return camera1394::Camera1394_OnePush;
    default:
      return camera1394::Camera1394_Off;
    }
}

/** Get feature values.
 *
 *  @pre feature_set_ initialized for this camera
 *
 *  @param finfo pointer to information for this feature
 *  @param value [out] pointer where parameter value stored
 *  @param value2 [out] optional pointer for second parameter value
 *               for white balance.  Otherwise NULL.
 */
void Features::getValues(dc1394feature_info_t *finfo,
                           double *value, double *value2)
{
  dc1394feature_t feature = finfo->id;
  dc1394error_t rc;

  if (!finfo->readout_capable)
    {
      ROS_INFO_STREAM("feature " << featureName(feature)
                      << " value not available from device");
      return;
    }

  if (feature == DC1394_FEATURE_WHITE_BALANCE)
    {
      // handle White Balance separately, it has two components
      if (finfo->absolute_capable && finfo->abs_control)
        {
          // supports reading and setting float value
          // @todo get absolute White Balance values
          rc = DC1394_FUNCTION_NOT_SUPPORTED;
        }
      else
        {
          // get integer White Balance values
          uint32_t bu_val;
          uint32_t rv_val;
          rc = dc1394_feature_whitebalance_get_value(camera_, &bu_val, &rv_val);
          if (DC1394_SUCCESS == rc)
            {
              // convert to double
              *value = bu_val;
              *value2 = rv_val;
            }
        }
      if (DC1394_SUCCESS == rc)
        {
          ROS_DEBUG_STREAM("feature " << featureName(feature)
                           << " Blue/U: " << *value
                           << " Red/V: " << *value2);
        }
      else
        {
          ROS_WARN_STREAM("failed to get values for feature "
                          << featureName(feature));
        }
    }
  else
    {
      // other features only have one component
      if (finfo->absolute_capable && finfo->abs_control)
        {
          // supports reading and setting float value
          float fval;
          rc = dc1394_feature_get_absolute_value(camera_, feature, &fval);
          if (DC1394_SUCCESS == rc)
            {
              *value = fval;                // convert to double
            }
        }
      else // no float representation
        {
          uint32_t ival;
          rc = dc1394_feature_get_value(camera_, feature, &ival);
          if (DC1394_SUCCESS == rc)
            {
              *value = ival;                // convert to double
            }
        }
      if (DC1394_SUCCESS == rc)
        {
          ROS_DEBUG_STREAM("feature " << featureName(feature)
                           << " has value " << *value);
        }
      else
        {
          ROS_WARN_STREAM("failed to get value of feature "
                          << featureName(feature));
        }
    }
}

/** Set mode for a feature.
 *
 *  @pre feature_set_ initialized for this camera
 *
 *  @param finfo pointer to information for this feature
 *  @param mode DC1394 mode desired
 *  @return true if mode set successfully
 */
bool Features::setMode(dc1394feature_info_t *finfo, dc1394feature_mode_t mode)
{
  dc1394feature_t feature = finfo->id;
  if (hasMode(finfo, mode))
    {
      // first, make sure the feature is powered on
      setPower(finfo, DC1394_ON);

      ROS_DEBUG_STREAM("setting feature " << featureName(feature)
                       << " mode to " << modeName(mode));
      if (DC1394_SUCCESS !=
          dc1394_feature_set_mode(camera_, feature, mode))
        {
          ROS_WARN_STREAM("failed to set feature " << featureName(feature)
                          << " mode to " << modeName(mode));
          return false;
        }
    }
  else
    {
      // device does not support this mode for this feature
      ROS_DEBUG_STREAM("no " << modeName(mode)
                       << " mode for feature " << featureName(feature));
      return false;
    }
  return true;
}

/** Set power for a feature On or Off.
 *
 *  @pre feature_set_ initialized for this camera
 *
 *  @param finfo pointer to information for this feature
 *  @param on_off either DC1394_ON or DC1394_OFF
 */
void Features::setPower(dc1394feature_info_t *finfo, dc1394switch_t on_off)
{
  dc1394feature_t feature = finfo->id;
  if (finfo->on_off_capable)
    {
      ROS_DEBUG_STREAM("Setting power for feature " << featureName(feature)
                       << " to " << on_off);
      if (DC1394_SUCCESS !=
          dc1394_feature_set_power(camera_, feature, on_off))
        {
          ROS_WARN_STREAM("failed to set feature " << featureName(feature)
                          << " power to " << on_off);
        }
    }
  else
    {
      // This device does not support turning this feature on or off.
      // That's OK.
      ROS_DEBUG_STREAM("no power control for feature " << featureName(feature));
    }
}

/** Update a feature for the currently open device, if changed.
 *
 *  Normal version: for features with one value parameter.
 *
 *  @pre feature_set_ initialized
 *  @pre oldconfig_ has previous settings
 *
 *  @param feature desired feature number
 *  @param old_control previous control parameter setting
 *  @param control [in,out] pointer to control parameter (may change)
 *  @param old_value previous parameter value
 *  @param value [in,out] pointer to requested parameter value (may
 *               change depending on device restrictions)
 */
void Features::updateIfChanged(dc1394feature_t feature,
                               int old_control, int *control,
                               double old_value, double *value)
{
  if ((old_control != *control) || (old_value != *value))
    {
      configure(feature, control, value);
    }
}

/** Update a feature for the currently open device, if changed.
 *
 *  Special version: for White Balance feature with two value parameters.
 *
 *  @pre feature_set_ initialized
 *  @pre oldconfig_ has previous settings
 *
 *  @param feature desired feature number
 *  @param old_control previous control parameter setting
 *  @param control [in,out] pointer to control parameter (may change)
 *  @param old_value previous parameter value
 *  @param value [in,out] pointer to requested parameter value (may
 *               change depending on device restrictions)
 *  @param old_value2 previous second parameter value
 *  @param value2 [in,out] pointer to requested second parameter value
 *               (may change depending on device restrictions).
 */
void Features::updateIfChanged(dc1394feature_t feature,
                               int old_control, int *control,
                               double old_value, double *value,
                               double old_value2, double *value2)
{
  if ((old_control != *control)
      || (old_value != *value)
      || (old_value2 != *value2))
    {
      configure(feature, control, value, value2);
    }
}
