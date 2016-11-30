#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/flash.h>

#include <uf_subbus_protocol/protocol.h>
#include <uf_subbus_protocol/sha256.h>

#include <arm_bootloader/protocol.h>

extern unsigned _begin_client_rom;
extern unsigned _end_client_rom;

namespace arm_bootloader {

static uint8_t *client_flash_start = reinterpret_cast<uint8_t*>(&_begin_client_rom);
static uint8_t *client_flash_end = reinterpret_cast<uint8_t*>(&_end_client_rom);

__attribute__((warn_unused_result))
static bool flash_erase(void *dest) {
  //assert(reinterpret_cast<uint32_t>(dest) % page_size == 0);
  
  FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PG); // stlink programmer seems to leave PG set
  FLASH_CR |= FLASH_CR_PER;
  FLASH_AR = reinterpret_cast<uint32_t>(dest);
  FLASH_CR |= FLASH_CR_STRT;
  flash_wait_for_last_operation();
  FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PER);
  if(!(FLASH_SR & FLASH_SR_EOP)) return false;
  flash_clear_eop_flag();
  return true;
}

__attribute__((warn_unused_result))
static bool flash_write(uint8_t volatile *dest, uint8_t const *src, size_t length_in_bytes) {
  assert(length_in_bytes % 2 == 0);
  
  FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PG); // stlink programmer seems to leave PG set
  for(uint32_t i = 0; i < length_in_bytes/2; i++) {
      FLASH_CR |= FLASH_CR_PG;
      reinterpret_cast<uint16_t volatile *>(dest)[i] =
        reinterpret_cast<uint16_t const *>(src)[i];
      flash_wait_for_last_operation();
      FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PG);
      if(!(FLASH_SR & FLASH_SR_EOP)) return false;
      flash_clear_eop_flag();
  }
  return true;
}

class Handler {
  class GotMessageFunctor {
    Handler &handler;
  public:
    GotMessageFunctor(Handler &handler) :
      handler(handler) {
    }
    void operator()(const Command &command) {
      handler.handle_message(command);
    }
  };
  
  GotMessageFunctor gmf;
  uf_subbus_protocol::SimpleReceiver<Command, GotMessageFunctor> receiver;
  uf_subbus_protocol::SimpleSender<Response, uf_subbus_protocol::ISink> sender;
  Dest dest;
  uint32_t page_size;

public:
  Handler(uf_subbus_protocol::ISink &sink, Dest dest, uint32_t page_size) :
    gmf(*this), receiver(gmf), sender(sink), dest(dest), page_size(page_size) {
  }
  
  void handleByte(uint8_t byte) {
    receiver.handleRawByte(byte);
  }
  
  void handle_message(const Command &msg) {
    if(msg.dest != dest) return;
    
    Response resp; memset(&resp, 0, sizeof(resp));
    resp.id = msg.id;
    
    switch(msg.command) {
    
      case CommandID::Reset: {
        // action happens later, after response is sent
      } break;
    
      case CommandID::GetStatus: {
        resp.resp.GetStatus.magic = GetStatusResponse::MAGIC_VALUE;
      } break;
      
      case CommandID::GetProgramHash: {
        if(msg.args.GetProgramHash.length > static_cast<int64_t>(client_flash_end - client_flash_start)) {
          resp.resp.GetProgramHash.error_number = 1;
          break;
        }
        
        uf_subbus_protocol::sha256_state md; uf_subbus_protocol::sha256_init(md);
        uf_subbus_protocol::sha256_process(md, client_flash_start, msg.args.GetProgramHash.length);
        
        resp.resp.GetProgramHash.error_number = 0;
        uf_subbus_protocol::sha256_done(md, resp.resp.GetProgramHash.hash);
      } break;
      
      case CommandID::Erase: {
        if(msg.args.Erase.min_length > client_flash_end - client_flash_start) {
          resp.resp.Erase.error_number = 1;
          break;
        }
        
        flash_unlock();
        
        uint8_t *to_erase = client_flash_start;
        bool failed = false;
        while(to_erase - client_flash_start < msg.args.Erase.min_length) {
          if(!flash_erase(to_erase)) {
            failed = true;
            break;
          }
          to_erase += page_size;
        }
        
        flash_lock();
        
        if(failed) {
          resp.resp.Erase.error_number = 2;
          break;
        }
        
        resp.resp.Erase.error_number = 0;
      } break;
      
      case CommandID::Flash: {
        if(msg.args.Flash.length > FlashCommand::MAX_LENGTH) {
          resp.resp.Flash.error_number = 1;
          break;
        }
        if(static_cast<int64_t>(msg.args.Flash.start_offset) +
            msg.args.Flash.length > client_flash_end - client_flash_start) {
          resp.resp.Flash.error_number = 2;
          break;
        }
        
        flash_unlock();
        bool success = flash_write(
          client_flash_start + msg.args.Flash.start_offset,
          msg.args.Flash.data, msg.args.Flash.length);
        flash_lock();
        if(!success) {
          resp.resp.Flash.error_number = 3;
          break;
        }
        
        resp.resp.Flash.error_number = 0;
      } break;

      case CommandID::RunProgram: {
        // action happens later, after response is sent
      } break;

      default: {
        return; // send nothing back if command is invalid
      } break;

    }
    
    if(resp.id) {
      sender.write_object(resp);
    }
    
    switch(msg.command) {
      case CommandID::Reset: {
        scb_reset_system();
      } break;

      case CommandID::RunProgram: {
        SCB_VTOR = reinterpret_cast<uint32_t>(client_flash_start);

        vector_table_t *new_vector_table = reinterpret_cast<vector_table_t*>(client_flash_start);

        asm("mov SP, %0;bx %1;" ::
          "r"(new_vector_table->initial_sp_value),
          "r"(new_vector_table->reset));
      } break;
      
      default: {
      } break;
    }
  }
};



}
