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

#ifndef _FEATURES_H_
#define _FEATURES_H_

#include <dc1394/dc1394.h>

#include "camera1394/Camera1394Config.h"
#include "trigger.h"
typedef camera1394::Camera1394Config Config;

/** @file

    @brief Camera1394 features interface

    @author Jack O'Quin
 */

/** @brief Camera1394 Features class

    Sets IIDC features from Config updates.  Tracks values and ranges,
    modifying configured values to those supported by the device.

*/

class Features
{
public:

  Features(dc1394camera_t *camera);
  ~Features() {};
  bool initialize(Config *newconfig);
  void reconfigure(Config *newconfig);

  inline bool isTriggerPowered()
  {
	  return trigger_->isPowered();
  }

private:
  typedef int state_t;      ///< camera1394::Camera1394_* state values

  void configure(dc1394feature_t feature, int *control,
                 double *value, double *value2=NULL);
  state_t getState(dc1394feature_info_t *finfo);
  void getValues(dc1394feature_info_t *finfo,
                 double *value, double *value2);

  /** Does this camera feature support a given mode?
   *
   *  @pre feature_set_ initialized for this camera
   *
   *  @param finfo pointer to information for this feature
   *  @param mode DC1394 mode desired
   *  @return true if mode supported
   */
  inline bool hasMode(dc1394feature_info_t *finfo, dc1394feature_mode_t mode)
  {
    for (uint32_t i = 0; i < finfo->modes.num; ++i)
      {
        if (finfo->modes.modes[i] == mode)
          return true;
      }
    return false;
  }

  /** Does this camera support triggering?
   *
   *  @pre feature_set_ initialized for this camera
   *  @return true if triggering supported
   */
  inline bool hasTrigger(void)
  {
    return DC1394_TRUE == feature_set_.feature[DC1394_FEATURE_TRIGGER
                                               - DC1394_FEATURE_MIN].available;
  }

  // pointer to subordinate trigger class
  boost::shared_ptr<Trigger> trigger_;

  bool setMode(dc1394feature_info_t *finfo, dc1394feature_mode_t mode);
  void setPower(dc1394feature_info_t *finfo, dc1394switch_t on_off);
  void updateIfChanged(dc1394feature_t feature,
                       int old_control, int *control,
                       double old_value, double *value);
  void updateIfChanged(dc1394feature_t feature,
                       int old_control, int *control,
                       double old_value, double *value,
                       double old_value2, double *value2);

  dc1394camera_t *camera_;              ///< current camera
  dc1394featureset_t feature_set_;      ///< that camera's feature set
  Config oldconfig_;                    ///< previous Config settings
};

#endif // _FEATURES_H_
