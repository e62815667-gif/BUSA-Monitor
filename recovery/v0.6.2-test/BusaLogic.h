#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Pure logic shared by firmware and host regression tests.
inline uint8_t crc8(const uint8_t* b, size_t n) {
  uint8_t crc=0;
  for(size_t i=0;i<n;i++) {
    uint8_t v=b[i];
    for(int j=0;j<8;j++) {
      uint8_t mix=(crc^v)&1; crc>>=1;
      if(mix) crc^=0x8C;
      v>>=1;
    }
  }
  return crc;
}
inline void incrementSaturated(uint32_t &v) { if(v!=UINT32_MAX) ++v; }
inline bool decodeTemperature(const uint8_t b[9], float &out) {
  if(crc8(b,8)!=b[8]) return false;
  // DS18B20 reserved configuration bits, reserved bytes 5 and 7.
  // Do not check byte 6: its value is not needed for DS18B20 decoding.
  if((b[4]&0x9F)!=0x1F || b[5]!=0xFF || b[7]!=0x10) return false;
  uint16_t bits=(uint16_t)b[0]|((uint16_t)b[1]<<8);
  uint8_t resolution=(b[4]>>5)&3;
  bits &= (uint16_t)(0xFFFFu << (3-resolution));
  int32_t raw=(bits&0x8000)?(int32_t)bits-65536:(int32_t)bits;
  float t=raw/16.0f;
  if(t < -55.0f || t > 125.0f) return false;
  out=t; return true; // 85 C is valid after a verified conversion.
}
struct Sensor {
  const char* id;
  const char* name;
  uint8_t channel;
  float value=NAN;
  bool valid=false, present=false, pending=false, hasSample=false;
  uint32_t updated=0, started=0, errors=0;
  const char* fault="not_ready";
  uint8_t rom[8]={0};
};
inline void invalidate(Sensor &s, const char* reason, bool count=true) {
  s.valid=false; s.value=NAN; s.pending=false; s.fault=reason;
  if(count) incrementSaturated(s.errors);
}
inline bool fresh(const Sensor &s, uint32_t now, uint32_t timeout) {
  return s.valid && s.hasSample && uint32_t(now-s.updated)<=timeout;
}
// -1 = released; -2 = multiple keys. No repeat while held.
struct KeyFilter {
  int candidate=-1, stable=-1;
  uint32_t since=0;
  bool blocked=false;
  void reset(uint32_t now) {candidate=stable=-1;since=now;blocked=false;}
  int feed(int raw, uint32_t now) {
    if(raw==-2) {blocked=true; stable=-1;}
    if(raw!=candidate) {candidate=raw;since=now;return -1;}
    if(uint32_t(now-since)<60) return -1;
    if(blocked) { if(raw==-1) {blocked=false;stable=-1;} return -1; }
    if(raw==stable) return -1;
    stable=raw; return raw>=0?raw:-1;
  }
};
struct KeyEvent { uint32_t seq=0, time=0; uint8_t key=0; };
struct KeyHistory {
  static constexpr uint8_t CAPACITY=64;
  KeyEvent items[CAPACITY];
  uint8_t head=0, count=0;
  uint32_t sequence=0, overwritten=0;
  // Caller changes stream_id on the practically unreachable counter rollover.
  bool push(uint8_t key, uint32_t now) {
    bool rolled=sequence==UINT32_MAX;
    if(rolled) {head=count=0;sequence=overwritten=0;}
    items[head]={++sequence,now,key};
    head=(head+1)%CAPACITY;
    if(count<CAPACITY) ++count; else incrementSaturated(overwritten);
    return rolled;
  }
  const KeyEvent& at(uint8_t i) const {return items[(head+CAPACITY-count+i)%CAPACITY];}
};
