#ifndef GUARD_QUARTZXBGSRPXCAU
#define GUARD_QUARTZXBGSRPXCAU

#include <libopencm3/stm32/memorymap.h>

#include <uf_subbus_protocol/sha256.h>

#include <arm_bootloader/protocol.h>

namespace arm_bootloader {


inline Dest get_unique_dest() {
  uf_subbus_protocol::sha256_state md; uf_subbus_protocol::sha256_init(md);
  
  uint32_t a = DESIG_UNIQUE_ID0;
  uf_subbus_protocol::sha256_process(md, reinterpret_cast<uint8_t*>(&a), sizeof(a));
  uint32_t b = DESIG_UNIQUE_ID1;
  uf_subbus_protocol::sha256_process(md, reinterpret_cast<uint8_t*>(&b), sizeof(b));
  uint32_t c = DESIG_UNIQUE_ID2;
  uf_subbus_protocol::sha256_process(md, reinterpret_cast<uint8_t*>(&c), sizeof(c));
  
  uint8_t hash[32]; uf_subbus_protocol::sha256_done(md, hash);
  Dest dest = 0;
  for(uint32_t i = 0; i < sizeof(dest); i++) {
    dest |= static_cast<Dest>(hash[i]) << (8*i);
  }
  return dest;
}


}

#endif

