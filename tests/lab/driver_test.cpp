#include <cassert>
#include <cstdio>
#include "../../firmware/v0.7.2/main/TemperatureBus.h"
uint32_t fakeMillis=0;
void prepare(){
 Wire=WireFake{};fakeMillis=0;busReady=true;bridgeFound=false;DS2482_ADDR=0x18;
 for(int i=0;i<4;i++){
  sensors[i]=Sensor{"T","test",(uint8_t)i};auto &s=Wire.dev[i];
  s.rom={0x28,(uint8_t)i,2,3,4,5,6,0};s.rom[7]=crc8(s.rom.data(),7);
  s.scratch={0x90,1,75,70,127,255,12,16,0};s.scratch[8]=crc8(s.scratch.data(),8);
 }assert(initBridge());
}
void cycle(){for(int i=0;i<110;i++){pollSensors();fakeMillis+=10;}}
int main(){
 prepare();cycle();for(auto &s:sensors)assert(s.valid&&s.value==25);
 prepare();Wire.dev[0].present=false;cycle();assert(!sensors[0].valid&&sensors[1].valid);
 prepare();Wire.dev[0].shorted=true;cycle();assert(!sensors[0].valid);
 prepare();Wire.dev[0].external=false;cycle();assert(!sensors[0].valid&&sensors[1].valid);
 prepare();Wire.dev[0].rom[7]^=1;cycle();assert(!sensors[0].valid);
 prepare();Wire.dev[0].scratch[8]^=1;cycle();assert(!sensors[0].valid);
 prepare();Wire.dev[0].stuck=true;cycle();assert(!sensors[0].valid);
 prepare();Wire.dev[0].earlyDone=true;cycle();assert(!sensors[0].valid);
 prepare();Wire.dev[0].failConvert=true;cycle();assert(!bridgeFound);
 prepare();Wire.badChannel=true;cycle();assert(!bridgeFound);
 prepare();Wire.badConfig=true;assert(!initBridge());
 prepare();Wire.status|=0x10;cycle();assert(!bridgeFound);
 prepare();cycle();Wire.bridge=false;cycle();for(auto &s:sensors)assert(!s.valid);
 Wire.bridge=true;assert(initBridge());cycle();assert(sensors[0].valid);
 prepare();Wire.dev[0].scratch[0]=0x50;Wire.dev[0].scratch[1]=5;
 Wire.dev[0].scratch[8]=crc8(Wire.dev[0].scratch.data(),8);cycle();assert(sensors[0].valid&&sensors[0].value==85);
 fakeMillis+=3100;assert(!fresh(sensors[0],fakeMillis,3000));
 Sensor wrap={"T","test",0};wrap.valid=wrap.hasSample=true;wrap.updated=UINT32_MAX-100;assert(fresh(wrap,50,3000));
 puts("PASS DS2482: 16 host fault/decoding scenarios; NOT hardware tests");
}

