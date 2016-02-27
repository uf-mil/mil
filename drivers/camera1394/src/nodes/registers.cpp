/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 Tomas Petricek
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

#include "registers.h"

/** @file

    @brief IEEE 1394 camera registers interface implementation

    @author Tomas Petricek
 */

namespace camera1394
{

Registers::Registers(dc1394camera_t *camera) : camera_(camera)
{}

Registers::Registers() : camera_(NULL)
{}

// Accessors for camera control and status registers (CSR),
// generally wrapping dc1394 functions.

/** Get camera control registers.
 *
 *  @param offset register offset
 *  @param num_regs number of registers to read
 *  @param val buffer to fill in (must be preallocated)
 *  @return true if succeeded
 */
bool Registers::getControlRegisters(const uint64_t offset,
                                    const uint32_t num_regs,
                                    std::vector<uint32_t> &val)
{
  int err = dc1394_get_control_registers(camera_, offset, &val[0], num_regs);
  return (DC1394_SUCCESS == err);
}
/** Set camera camera registers.
 *
 *  @param offset register offset
 *  @param val values to set
 *  @return true if succeeded
 */
bool Registers::setControlRegisters(const uint64_t offset,
                                    const std::vector<uint32_t> &val)
{
  int err = dc1394_set_control_registers(camera_, offset, &val[0], val.size());
  return (DC1394_SUCCESS == err);
}

/** Get feature absolute value register. */
bool Registers::getAbsoluteRegister(const uint64_t offset,
                                    const uint32_t feature,
                                    uint32_t &val)
{
  int err = dc1394_get_absolute_register(camera_, feature, offset, &val);
  return (DC1394_SUCCESS == err);
}
/** Set feature absolute value register. */
bool Registers::setAbsoluteRegister(const uint64_t offset,
                                    const uint32_t feature,
                                    const uint32_t val)
{
  int err = dc1394_set_absolute_register(camera_, feature, offset, val);
  return (DC1394_SUCCESS == err);
}

/** Get Format7 register. */
bool Registers::getFormat7Register(const uint64_t offset,
                                   const uint32_t mode,
                                   uint32_t &val)
{
  int err = dc1394_get_format7_register(camera_, mode, offset, &val);
  return (DC1394_SUCCESS == err);
}
/** Set Format7 register. */
bool Registers::setFormat7Register(const uint64_t offset,
                                   const uint32_t mode,
                                   const uint32_t val)
{
  int err = dc1394_set_format7_register(camera_, mode, offset, val);
  return (DC1394_SUCCESS == err);
}

/** Get advanced feature registers. */
bool Registers::getAdvancedControlRegisters(const uint64_t offset,
                                            const uint32_t num_regs,
                                            std::vector<uint32_t> &val)
{
  int err = dc1394_get_adv_control_registers(camera_, offset, &val[0],
                                             num_regs);
  return (DC1394_SUCCESS == err);
}
/** Set advanced feature registers. */
bool Registers::setAdvancedControlRegisters(const uint64_t offset,
                                            const std::vector<uint32_t> &val)
{
  int err = dc1394_set_adv_control_registers(camera_, offset, &val[0],
                                             val.size());
  return (DC1394_SUCCESS == err);
}

/** Get parallel input/output (PIO) register. */
bool Registers::getPIORegister(const uint64_t offset, uint32_t &val)
{
  int err = dc1394_get_PIO_register(camera_, offset, &val);
  return (DC1394_SUCCESS == err);
}
/** Set parallel input/output (PIO) register. */
bool Registers::setPIORegister(const uint64_t offset, const uint32_t val)
{
  int err = dc1394_set_PIO_register(camera_, offset, val);
  return (DC1394_SUCCESS == err);
}

/** Get serial input/output (SIO) register. */
bool Registers::getSIORegister(const uint64_t offset, uint32_t &val)
{
  int err = dc1394_get_SIO_register(camera_, offset, &val);
  return (DC1394_SUCCESS == err);
}
/** Set serial input/output (SIO) register. */
bool Registers::setSIORegister(const uint64_t offset, const uint32_t val)
{
  int err = dc1394_set_SIO_register(camera_, offset, val);
  return (DC1394_SUCCESS == err);
}

/** Get strobe register. */
bool Registers::getStrobeRegister(const uint64_t offset,uint32_t &val)
{
  int err = dc1394_get_strobe_register(camera_, offset, &val);
  return (DC1394_SUCCESS == err);
}
/** Set strobe register. */
bool Registers::setStrobeRegister(const uint64_t offset, const uint32_t val)
{
  int err = dc1394_set_strobe_register(camera_, offset, val);
  return (DC1394_SUCCESS == err);
}

} // namespace
