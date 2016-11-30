#ifndef _FBDARVFXLZPFVOBY_
#define _FBDARVFXLZPFVOBY_

#include <uf_subbus_protocol/sha256.h>

namespace uf_subbus_protocol {

class ISink {
public:
  virtual void handleStart() = 0;
  virtual void handleByte(uint8_t byte) = 0;
  virtual void handleEnd() = 0;
};

static uint8_t const ESCAPE = 0x34;
static uint8_t const ESCAPE_START = 0x01;
static uint8_t const ESCAPE_ESCAPE = 0x02;
static uint8_t const ESCAPE_END = 0x03;

template<typename InnerType>
class Packetizer {
  InnerType &inner;
  
public:
  Packetizer(InnerType &inner) : inner(inner) {
  }
  
  void handleStart() {
    inner.handleStart();
    inner.handleByte(ESCAPE);
    inner.handleByte(ESCAPE_START);
  }
  void handleByte(uint8_t byte) {
    if(byte == ESCAPE) {
      inner.handleByte(ESCAPE);
      inner.handleByte(ESCAPE_ESCAPE);
    } else {
      inner.handleByte(byte);
    }
  }
  void handleEnd() {
    inner.handleByte(ESCAPE);
    inner.handleByte(ESCAPE_END);
    inner.handleEnd();
  }
};

template<typename InnerType>
class Depacketizer {
  InnerType &inner;
  bool in_message;
  bool in_escape;
  
public:
  Depacketizer(InnerType &inner) : inner(inner),
    in_message(false), in_escape(false) {
  }
  
  void handleRawByte(uint8_t byte) {
    if(byte == ESCAPE) {
      if(in_escape) {
        in_message = false; // shouldn't happen, reset
      }
      
      in_escape = true;
    
    } else if(in_escape) {
      in_escape = false;
      
      if(byte == ESCAPE_START) {
        inner.handleStart();
        in_message = true;
      
      } else if(byte == ESCAPE_ESCAPE && in_message) {
        inner.handleByte(ESCAPE);
      
      } else if(byte == ESCAPE_END && in_message) {
        inner.handleEnd();
        in_message = false;
      
      } else {
        in_message = false; // shouldn't happen, reset
      }
    
    } else {
      if(in_message) {
        inner.handleByte(byte);
      
      } else {
        // shouldn't happen, ignore
      }
    }
  }
};



template<typename InnerType>
class ChecksumAdder {
  InnerType &inner;
  sha256_state md;
  
public:
  ChecksumAdder(InnerType &inner) : inner(inner) {
  }
  
  void handleStart() {
    sha256_init(md);
    inner.handleStart();
  }
  
  void handleByte(uint8_t byte) {
    sha256_process(md, &byte, 1);
    inner.handleByte(byte);
  }
  
  void handleEnd() {
    uint8_t hash[32];
    sha256_done(md, hash);
    
    for(uint8_t i = 0; i < 4; i++) {
      inner.handleByte(hash[i]);
    }
    
    inner.handleEnd();
  }
};

template<typename InnerType>
class ChecksumChecker {
  InnerType &inner;
  uint8_t buf[4];
  uint8_t buf_len;
  sha256_state md;
  
public:
  ChecksumChecker(InnerType &inner) : inner(inner) {
  }
  
  void handleStart() {
    buf_len = 0;
    sha256_init(md);
    inner.handleStart();
  }
  
  void handleByte(uint8_t byte) {
    if(buf_len == 4) {
      inner.handleByte(buf[0]);
      sha256_process(md, &buf[0], 1);
      for(uint8_t i = 0; i < 3; i++) {
        buf[i] = buf[i+1];
      }
      buf_len--;
    }
    
    buf[buf_len++] = byte;
  }
  
  void handleEnd() {
    if(buf_len != 4) return;
    
    uint8_t hash[32];
    sha256_done(md, hash);
    
    for(uint8_t i = 0; i < 4; i++) {
      if(buf[i] != hash[i]) return;
    }
    
    inner.handleEnd();
  }
};



template<typename ObjectType, typename InnerType>
void write_object(ObjectType const &obj, InnerType &inner) {
  inner.handleStart();
  uint8_t const *end = reinterpret_cast<uint8_t const*>(&obj) + sizeof(ObjectType);
  while(end > reinterpret_cast<uint8_t const*>(&obj) && *(end - 1) == 0) {
    end--;
  }
  for(uint8_t const *p = reinterpret_cast<uint8_t const*>(&obj); p < end; p++) {
    inner.handleByte(*p);
  }
  inner.handleEnd();
};

template<typename ObjectType, typename CallbackType>
class ObjectReceiver {
  ObjectType obj;
  uint32_t obj_pos;
  CallbackType &callback;

public:
  ObjectReceiver(CallbackType &callback) : callback(callback) {
  }
  
  void handleStart() {
    obj_pos = 0;
    memset(&obj, 0, sizeof(ObjectType));
  }
  void handleByte(uint8_t byte) {
    if(obj_pos < sizeof(obj)) {
      *(reinterpret_cast<uint8_t *>(&obj) + obj_pos) = byte;
    }
    obj_pos++;
  }
  void handleEnd() {
    if(obj_pos > sizeof(obj)) return; // overran obj
    callback(static_cast<ObjectType const>(obj));
  }
};

template<typename ObjectType>
class ObjectHolder {
public:
  bool is_set;
  ObjectType obj;
  ObjectHolder() : is_set(false) { }
  void set(ObjectType const &obj) {
    this->obj = obj;
    is_set = true;
  }
  void clear() {
    is_set = false;
  }
};


template<typename ObjectType, typename SinkType>
class SimpleSender {
  Packetizer<SinkType> packetizer;
  ChecksumAdder<Packetizer<SinkType> > checksumadder;
  
public:
  SimpleSender(SinkType &sink) :
    packetizer(sink), checksumadder(packetizer) {
  }
  void write_object(ObjectType const &obj) {
    uf_subbus_protocol::write_object(obj, checksumadder);
  }
};

template<typename ObjectType, typename CallbackType>
class SimpleReceiver {
  ObjectReceiver<ObjectType, CallbackType> objectreceiver;
  ChecksumChecker<ObjectReceiver<ObjectType, CallbackType> >
    cc;
  Depacketizer<ChecksumChecker<ObjectReceiver<ObjectType, CallbackType> > >
    depacketizer;

public:
  SimpleReceiver(CallbackType &messageReceived) :
    objectreceiver(messageReceived), cc(objectreceiver), depacketizer(cc) {
  }
  
  void handleRawByte(uint8_t byte) {
    depacketizer.handleRawByte(byte);
  }
};


}

#endif
