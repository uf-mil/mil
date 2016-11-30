#ifndef _BEZPKWLJADSOEQBW_
#define _BEZPKWLJADSOEQBW_

#include <stdint.h>

namespace arm_bootloader {


enum class CommandID : uint16_t {
  // commands that everything should implement
  Reset = 0,
  GetStatus,
  
  // commands specific to this firmware
  GetProgramHash = 0xf9c3,
  Erase,
  Flash,
  RunProgram,
};


struct __attribute__((packed)) ResetCommand {
};
struct __attribute__((packed)) ResetResponse {
};

struct __attribute__((packed)) GetStatusCommand {
};
struct __attribute__((packed)) GetStatusResponse {
  uint64_t magic; static uint64_t const MAGIC_VALUE = 0x45f5488f98180f73;
};


struct __attribute__((packed)) GetProgramHashCommand {
  uint32_t length;
};
struct __attribute__((packed)) GetProgramHashResponse {
  uint8_t error_number; // 0 means success
  uint8_t hash[32];
};

struct __attribute__((packed)) EraseCommand {
  // erase at least (min_length) bytes from the start
  uint32_t min_length;
};
struct __attribute__((packed)) EraseResponse {
  uint8_t error_number; // 0 means success
};

struct __attribute__((packed)) FlashCommand {
  // write (length) bytes of (data) to (start_offset)
  static uint32_t const MAX_LENGTH = 2048;
  uint32_t start_offset;
  uint32_t length;
  uint8_t data[MAX_LENGTH];
};
struct __attribute__((packed)) FlashResponse {
  uint8_t error_number; // 0 means success
};

struct __attribute__((packed)) RunProgramCommand {
};
struct __attribute__((packed)) RunProgramResponse {
};

typedef uint32_t Dest;
typedef uint16_t ID;

struct __attribute__((packed)) Command {
  Dest dest;
  ID id; // 0 means don't send a response
  CommandID command;
  union {
    ResetCommand Reset;
    GetStatusCommand GetStatus;
    GetProgramHashCommand GetProgramHash;
    EraseCommand Erase;
    FlashCommand Flash;
    RunProgramCommand RunProgram;
  } args;
};

struct __attribute__((packed)) Response {
  ID id;
  union {
    ResetResponse Reset;
    GetStatusResponse GetStatus;
    GetProgramHashResponse GetProgramHash;
    EraseResponse Erase;
    FlashResponse Flash;
    RunProgramResponse RunProgram;
  } resp;
};


}

#endif
