#ifndef _JQJBADTGEGIIVPYH_
#define _JQJBADTGEGIIVPYH_

#include <stdint.h>

#include <arm_bootloader/protocol.h>

namespace navigator_emergency_control
{
enum class CommandID : uint16_t
{
  // commands that everything should implement
  Reset = 0,
  GetStatus,

  // commands specific to this firmware
  GetIMUData = 0x6393,
  SetPWM,
};

struct __attribute__((packed)) ResetCommand
{
};
struct __attribute__((packed)) ResetResponse
{
};

struct __attribute__((packed)) GetStatusCommand
{
};
struct __attribute__((packed)) GetStatusResponse
{
  uint64_t magic;
  static uint64_t const MAGIC_VALUE = 0xb67f739fff9612cd;
};

struct __attribute__((packed)) GetIMUDataCommand
{
};
struct __attribute__((packed)) GetIMUDataResponse
{
  uint64_t timestamp;             // ns
  double linear_acceleration[3];  // m/s^2
  double angular_velocity[3];     // rad/s
  double magnetic_field[3];       // T
};

struct __attribute__((packed)) SetPWMCommand
{
  float length[2];
};
struct __attribute__((packed)) SetPWMResponse
{
  float joy[4];
  bool buttons[20];
};

typedef uint16_t ID;

struct __attribute__((packed)) Command
{
  arm_bootloader::Dest dest;
  ID id;  // 0 means don't send a response
  CommandID command;
  union {
    ResetCommand Reset;
    GetStatusCommand GetStatus;
    GetIMUDataCommand GetIMUData;
    SetPWMCommand SetPWM;
  } args;
};

struct __attribute__((packed)) Response
{
  ID id;
  uint16_t _padding;  // needed to ensure 32-bit alignment
  union {
    ResetResponse Reset;
    GetStatusResponse GetStatus;
    GetIMUDataResponse GetIMUData;
    SetPWMResponse SetPWM;
  } resp;
};
}

#endif
