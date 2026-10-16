#pragma once
// DS2482 protocol recovered from v0.6.2-test. No keypad/LED/relay writes.
#include <Wire.h>
#include "BusaLogic.h"
static const uint32_t SAMPLE_MAX_AGE_MS=3000;
static const uint8_t BRIDGE_CONFIG=0x01;
uint8_t DS2482_ADDR=0x18;
Sensor sensors[4]={{"T1","Вода",0},{"T2","Колонна",1},
 {"T3","Куб",2},{"T4","Верхняя точка колонны",3}};
bool bridgeFound=false, busReady=false, readingPhase=false;
uint8_t sensorIndex=0;
uint32_t lastBridgeAttempt=0;
void bridgeOffline(const char* reason) {
  bridgeFound=false; readingPhase=false; sensorIndex=0;
  lastBridgeAttempt=millis();
  for(auto &s:sensors) {s.present=false; invalidate(s,reason);}
}
bool dsWrite(uint8_t cmd, int data=-1) {
  if(!busReady) return false;
  Wire.beginTransmission(DS2482_ADDR); Wire.write(cmd);
  if(data>=0) Wire.write((uint8_t)data);
  if(Wire.endTransmission()!=0) {bridgeOffline("bridge_i2c"); return false;}
  return true;
}
bool dsRead(uint8_t ptr, uint8_t &value) {
  if(!dsWrite(0xE1,ptr)) return false;
  if(Wire.requestFrom((uint8_t)DS2482_ADDR,(size_t)1)!=1) {
    bridgeOffline("bridge_i2c"); return false;
  }
  value=(uint8_t)Wire.read(); return true;
}
bool busyWait(uint8_t &status) {
  uint32_t start=millis();
  do {
    if(!dsRead(0xF0,status)) return false;
    if(status&0x10) {bridgeOffline("bridge_reset");return false;}
    if(!(status&1)) return true;
    delay(1);
  } while(uint32_t(millis()-start)<20);
  bridgeOffline("bridge_busy_timeout"); return false;
}
bool initBridge() {
  uint8_t status=0, cfg=0;
  if(!dsWrite(0xF0) || !dsRead(0xF0,status)) return false;
  if(!(status&0x10) || (status&1)) {bridgeOffline("bridge_reset_check");return false;}
  uint8_t encoded=(uint8_t)(BRIDGE_CONFIG|((~BRIDGE_CONFIG&15)<<4));
  if(!dsWrite(0xD2,encoded) || !dsRead(0xC3,cfg)) return false;
  if(cfg!=BRIDGE_CONFIG) {bridgeOffline("bridge_config");return false;}
  if(!busyWait(status)) return false;
  bridgeFound=true; readingPhase=false; sensorIndex=0;
  return true;
}
bool selectChannel(uint8_t ch) {
  static const uint8_t codes[8]={0xF0,0xE1,0xD2,0xC3,0xB4,0xA5,0x96,0x87};
  static const uint8_t replies[8]={0xB8,0xB1,0xAA,0xA3,0x9C,0x95,0x8E,0x87};
  if(ch>=8) return false;
  uint8_t status=0, actual=0;
  if(!busyWait(status)||!dsWrite(0xC3,codes[ch])||!dsRead(0xD2,actual)) return false;
  if(actual!=replies[ch]) {bridgeOffline("bridge_channel");return false;}
  return true;
}
bool owReset(Sensor &s) {
  uint8_t status=0;
  if(!busyWait(status)||!dsWrite(0xB4)||!busyWait(status)) return false;
  s.present=(status&2)!=0;
  if(status&4) {s.present=false;invalidate(s,"onewire_short");return false;}
  if(!s.present) {invalidate(s,"sensor_missing");return false;}
  return true;
}
bool owWrite(uint8_t value) {
  uint8_t status=0;
  return busyWait(status)&&dsWrite(0xA5,value)&&busyWait(status);
}
bool owRead(uint8_t &value) {
  uint8_t status=0;
  return busyWait(status)&&dsWrite(0x96)&&busyWait(status)&&dsRead(0xE1,value);
}
bool owBit(bool &one) {
  uint8_t status=0;
  if(!busyWait(status)||!dsWrite(0x87,0x80)||!busyWait(status)) return false;
  one=(status&0x20)!=0; return true;
}
bool skipRom(Sensor &s) {return owReset(s)&&owWrite(0xCC);}

void startSensor(Sensor &s) {
  s.pending=false;
  if(!selectChannel(s.channel)||!owReset(s)||!owWrite(0x33)) return;
  uint8_t rom[8];
  for(uint8_t i=0;i<8;i++) if(!owRead(rom[i])) return;
  if(rom[0]!=0x28 || crc8(rom,7)!=rom[7]) {invalidate(s,"rom_invalid");return;}
  for(uint8_t i=0;i<8;i++) s.rom[i]=rom[i];
  // Only external VDD is supported: simultaneous conversion on four channels.
  bool externallyPowered=false;
  if(!skipRom(s)||!owWrite(0xB4)||!owBit(externallyPowered)) return;
  if(!externallyPowered) {invalidate(s,"parasite_power_unsupported");return;}
  if(!skipRom(s)||!owWrite(0x44)) return;
  s.started=millis();
  bool done=true;
  if(!owBit(done)) return;
  if(done) {invalidate(s,"conversion_not_started");return;}
  s.pending=true;
}
void finishSensor(Sensor &s) {
  if(!s.pending) return; // Never accept old scratchpad after a failed start.
  if(!selectChannel(s.channel)) return;
  bool done=false;
  if(!owBit(done)) return;
  if(!done) {invalidate(s,"conversion_timeout");return;}
  if(!skipRom(s)||!owWrite(0xBE)) return;
  uint8_t scratch[9];
  for(uint8_t i=0;i<9;i++) if(!owRead(scratch[i])) return;
  float value=NAN;
  if(!decodeTemperature(scratch,value)) {invalidate(s,"scratchpad_invalid");return;}
  s.value=value; s.valid=true; s.pending=false; s.hasSample=true;
  s.updated=millis(); s.fault="none";
}
void pollSensors() {
  uint32_t now=millis();
  for(auto &s:sensors) if(s.valid&&!fresh(s,now,SAMPLE_MAX_AGE_MS)) invalidate(s,"stale");
  if(!bridgeFound) return;
  Sensor &s=sensors[sensorIndex];
  if(!readingPhase) {
    startSensor(s);
    if(!bridgeFound) return;
    if(++sensorIndex==4) {sensorIndex=0;readingPhase=true;}
  } else {
    if(s.pending && uint32_t(now-s.started)<800) return;
    finishSensor(s);
    if(!bridgeFound) return;
    if(++sensorIndex==4) {sensorIndex=0;readingPhase=false;}
  }
}

