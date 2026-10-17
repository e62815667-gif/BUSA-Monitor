#pragma once
// BUSA-1M-001 low-voltage PCF8575 4x4 keypad monitor. Not a relay interface.
// Matrix map is provisional: P00..P03 rows, P04..P07 columns.
// Wire levels/pin order MUST be confirmed before arming.
#include <Arduino.h>
#include <Wire.h>
#include "BusaLogic.h"
struct KeyboardMonitor {
  uint8_t address=0x20;
  bool armed=false, online=false;
  uint32_t lastAttempt=0,lastScan=0;
  KeyFilter filter;
  KeyHistory history;
  int lastPressed=-1;
  static const uint8_t ROW[4];
  static const uint8_t COL[4];
  static const char* keyText(uint8_t code) {
    static const char* names[16]={"1","2","3","A","4","5","6","B",
      "7","8","9","C","*","0","#","D"};
    return code<16?names[code]:"";
  }
  bool writePins(uint16_t p) {
    Wire.beginTransmission(address);
    Wire.write(uint8_t(p&255)); Wire.write(uint8_t(p>>8));
    return Wire.endTransmission()==0;
  }
  bool readPins(uint16_t& p) {
    if(Wire.requestFrom(address,(size_t)2)!=2) return false;
    p=(uint8_t)Wire.read();
    p|=(uint16_t)(uint8_t)Wire.read()<<8;
    return true;
  }
  void disarm() {
    if(armed) writePins(0xFFFF);
    armed=false;online=false;lastPressed=-1;
    filter.reset(millis());
  }
  bool start(uint8_t addr) {
    disarm();
    if(addr<0x20||addr>0x27) return false;
    address=addr;
    uint16_t v=0;
    bool ok=writePins(0xFFFF)&&readPins(v);
    armed=ok;online=ok;lastAttempt=millis();
    filter.reset(millis());return ok;
  }
  void poll() {
    if(!armed) return;
    uint32_t now=millis();
    if(uint32_t(now-lastScan)<20) return;
    lastScan=now;int found=-1;bool ok=true;
    for(uint8_t r=0;r<4;r++) {
      uint16_t state=0;
      if(!writePins(uint16_t(0xFFFFu & ~(1u<<ROW[r])))) {ok=false;break;}
      delayMicroseconds(250);
      if(!readPins(state)) {ok=false;break;}
      for(uint8_t c=0;c<4;c++) if(!(state&(1u<<COL[c]))) {
        found=found==-1?(int)(r*4+c):-2;
      }
    }
    if(!writePins(0xFFFF))ok=false;
    if(!ok){online=false;lastPressed=-1;filter.reset(now);return;}
    online=true;
    int k=filter.feed(found,now);
    if(k>=0) {lastPressed=k;history.push((uint8_t)k,now);}
  }
};
inline const uint8_t KeyboardMonitor::ROW[4]={0,1,2,3};
inline const uint8_t KeyboardMonitor::COL[4]={4,5,6,7};
