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

#ifndef CAMERA1394_REGISTERS_H
#define CAMERA1394_REGISTERS_H

#include <dc1394/dc1394.h>
#include <vector>

/** @file

    @brief IEEE 1394 camera registers interface

    @author Tomas Petricek
 */

namespace camera1394
{

/** @brief Registers class

    Allows to to get/set control and status registers (CSR).
 */
class Registers
{
public:
  Registers(dc1394camera_t *camera);
  Registers();

  bool getControlRegisters(const uint64_t offset, const uint32_t num_regs,
                           std::vector<uint32_t> &val);
  bool setControlRegisters(const uint64_t offset,
                           const std::vector<uint32_t> &val);

  bool getAbsoluteRegister(const uint64_t offset, const uint32_t feature,
                           uint32_t &val);
  bool setAbsoluteRegister(const uint64_t offset, const uint32_t feature,
                           const uint32_t val);

  bool getFormat7Register(const uint64_t offset, const uint32_t mode,
                          uint32_t &val);
  bool setFormat7Register(const uint64_t offset, const uint32_t mode,
                          const uint32_t val);

  bool getAdvancedControlRegisters(const uint64_t offset,
                                   const uint32_t num_regs,
                                   std::vector<uint32_t> &val);
  bool setAdvancedControlRegisters(const uint64_t offset,
                                   const std::vector<uint32_t> &val);

  bool getPIORegister(const uint64_t offset, uint32_t &val);
  bool setPIORegister(const uint64_t offset, const uint32_t val);

  bool getSIORegister(const uint64_t offset, uint32_t &val);
  bool setSIORegister(const uint64_t offset, const uint32_t val);

  bool getStrobeRegister(const uint64_t offset, uint32_t &val);
  bool setStrobeRegister(const uint64_t offset, const uint32_t val);

private:
  dc1394camera_t *camera_;
};
}

#endif // CAMERA1394_REGISTERS_H
