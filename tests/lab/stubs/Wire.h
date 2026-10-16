#pragma once
#include "Arduino.h"
#include <vector>
#include <array>
struct FakeSensor {
  bool present=true,shorted=false,external=true,stuck=false,earlyDone=false;
  bool failConvert=false;
  uint32_t started=0;
  uint8_t mode=0, pos=0;
  std::array<uint8_t,9> scratch{};
  std::array<uint8_t,8> rom{};
};
struct WireFake {
  bool bridge=true,pcf=true,badChannel=false,badConfig=false;
  uint8_t addr=0,ptr=0xF0,status=0x10,cfg=0,channel=0,data=0;
  uint16_t pins=0xFFFF;
  int key=-1, pcfReadCount=0, failPCFReadAt=-1;
  std::array<FakeSensor,4> dev;
  std::vector<uint8_t> tx,rx;
  bool begin(int,int,uint32_t){return true;}
  bool end(){return true;}
  void setTimeOut(int){}
  void beginTransmission(uint8_t a){addr=a;tx.clear();}
  size_t write(uint8_t v){tx.push_back(v);return 1;}
  uint8_t endTransmission(){
    if(addr==0x20){if(!pcf)return 2;pins=tx[0]|(tx[1]<<8);return 0;}
    if(!bridge)return 2;
    uint8_t c=tx[0];
    if(c==0xF0){status=0x18;cfg=0;channel=0;return 0;}
    if(c==0xE1){ptr=tx[1];return 0;}
    if(c==0xD2){cfg=tx[1]&15;status=8;return 0;}
    if(c==0xC3){channel=tx[1]&15;return 0;}
    auto &s=dev[channel];
    if(c==0xB4){status=8|(s.present?2:0)|(s.shorted?4:0);s.mode=0;return 0;}
    if(c==0xA5){
      uint8_t v=tx[1];
      if(v==0x44 && s.failConvert)return 3;
      if(v==0x33 || v==0xBE || v==0xB4){s.mode=v;s.pos=0;}
      if(v==0x44){s.mode=v;s.started=fakeMillis;}
      return 0;
    }
    if(c==0x96){
      data=s.mode==0x33?s.rom[s.pos++]:s.scratch[s.pos++];return 0;
    }
    if(c==0x87){
      bool one=s.mode==0xB4?s.external:(s.earlyDone||(!s.stuck&&uint32_t(fakeMillis-s.started)>=750));
      status=(status&~32)|(one?32:0);return 0;
    }
    return 0;
  }
  size_t requestFrom(uint8_t a,size_t n){
    rx.clear();
    if(a==0x20){
      ++pcfReadCount;if(!pcf || pcfReadCount==failPCFReadAt)return 0;
      uint16_t state=pins;
      if(key>=0 && !(pins&(1u<<(key/4))))state&=~(1u<<(5+key%4));
      rx={(uint8_t)state,(uint8_t)(state>>8)};return n;
    }
    if(!bridge)return 0;
    static const uint8_t channels[8]={0xB8,0xB1,0xAA,0xA3,0x9C,0x95,0x8E,0x87};
    uint8_t v=ptr==0xF0?status:ptr==0xC3?(badConfig?0:cfg):ptr==0xD2?(badChannel?0:channels[channel]):data;
    rx={v};return n;
  }
  int read(){int v=rx.front();rx.erase(rx.begin());return v;}
};
inline WireFake Wire;

