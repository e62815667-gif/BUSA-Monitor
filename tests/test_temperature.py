"""Execute the shipping temperature controller against a DS18B20 bus simulator."""
from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <cassert>
#include <cstdio>
#include <iostream>
using std::isfinite;
class String:public std::string{public:using std::string::string;String(const std::string&s):std::string(s){}String(int x):std::string(std::to_string(x)){}String(unsigned x):std::string(std::to_string(x)){}String(float x,int){char b[32];snprintf(b,sizeof b,"%.2f",x);assign(b);}};
String quoteJson(const String& s){return "\""+s+"\"";}
uint32_t nowMs=1;uint32_t millis(){return nowMs;}
bool elapsed(uint32_t a,uint32_t b,uint32_t c){return uint32_t(a-b)>=c;}
constexpr uint8_t TPIN[4]={16,17,18,21};
struct SimDevice{std::array<uint8_t,8> rom;std::array<uint8_t,9> scratch;bool external=true;};
struct OneWire{
 std::vector<SimDevice> devices;size_t searchAt=0;int selected=-1,readAt=0;bool low=false,failReset=false,conversionBusy=false;uint8_t lastCommand=0;int operations=0,converts=0,powerWrites=0;uint32_t lastConvert=0;
 static uint8_t crc8(const uint8_t* data,size_t n){uint8_t crc=0;while(n--){uint8_t b=*data++;for(int i=0;i<8;i++){uint8_t mix=(crc^b)&1;crc>>=1;if(mix)crc^=0x8c;b>>=1;}}return crc;}
 int reset(){++operations;return !low&&!failReset&&!devices.empty();}
 void reset_search(){searchAt=0;}
 bool search(uint8_t* out){++operations;if(searchAt>=devices.size()||low||failReset)return false;memcpy(out,devices[searchAt++].rom.data(),8);return true;}
 void select(const uint8_t* rom){++operations;selected=-1;for(size_t i=0;i<devices.size();i++)if(!memcmp(rom,devices[i].rom.data(),8))selected=i;}
 void skip(){++operations;selected=-1;}
 void write(uint8_t command,uint8_t power=0){++operations;lastCommand=command;assert(power==0);if(command==0x44){++converts;lastConvert=nowMs;}if(command==0xBE)readAt=0;assert(command==0x44||command==0xBE||command==0xB4);}
 uint8_t read(){++operations;assert(selected>=0&&readAt<9);return devices[selected].scratch[readAt++];}
 uint8_t read_bit(){++operations;return lastCommand==0x44?!conversionBusy:(selected>=0&&devices[selected].external);}
};
OneWire buses[4];OneWire* const temperatureBuses[4]={&buses[0],&buses[1],&buses[2],&buses[3]};
int digitalRead(int gpio){for(int i=0;i<4;i++)if(TPIN[i]==gpio)return !buses[i].low;return 1;}
'''
tests=r'''
SimDevice sensor(uint8_t serial,float c){
 SimDevice d;d.rom={0x28,serial,2,3,4,5,6,0};d.rom[7]=OneWire::crc8(d.rom.data(),7);
 int16_t raw=(int16_t)(c*16);d.scratch={(uint8_t)raw,(uint8_t)(raw>>8),75,70,0x7f,0xff,0x10,0x10,0};d.scratch[8]=OneWire::crc8(d.scratch.data(),8);return d;
}
void run(uint32_t duration){uint32_t end=nowMs+duration;while((int32_t)(nowMs-end)<0){
 int before=0;for(auto& b:buses)before+=b.operations;
 temperatureTick(nowMs);
 for(int i=0;i<4;i++)if(hasTemp(i,nowMs)){
   const auto& sample=wireProbe[tempBusForLogical(i)].devices[tempDeviceForLogical(i)];
   assert(isfinite(temperature[i]) && temperature[i]==sample.value && tempAt[i]==sample.sampledAt);
 }
 int after=0;for(auto& b:buses)after+=b.operations;
 // Each call services one bus operation; scratchpad read is at most 13 primitives.
 assert(after-before<=14);nowMs+=5;
}}
void restart(){for(auto& p:wireProbe)p=OneWireProbe();for(auto& st:tempBus)st=TempBusState();for(auto& bind:tempBindings)bind=TempBinding();tempNextBus=0;for(auto& b:buses){b.searchAt=0;b.selected=-1;b.readAt=0;b.low=b.failReset=false;}tempSyncLogical(nowMs);}
int main(){
 buses[0].devices={sensor(1,26.5)};buses[1].devices={sensor(2,31),sensor(3,42)};buses[3].devices={sensor(4,25.75)};
 run(700);assert(!hasTemp(0,nowMs));run(1100);
 assert(hasTemp(0,nowMs)&&temperature[0]==26.5f);assert(hasTemp(3,nowMs));
 assert(!hasTemp(1,nowMs)&&wireError(1,nowMs)=="MULTIPLE_ROM");assert(wireError(2,nowMs)=="NO_PRESENCE");
 // Inventory contains both real devices on GPIO17; no automatic role guessing.
 assert(wireProbe[1].count==2);assert(temperatureInventoryJSON(nowMs).find("42.00")!=std::string::npos);
 TempBinding binds[4];for(int i=0;i<4;i++)binds[i]=TempBinding();
 binds[1].explicitRom=binds[2].explicitRom=true;binds[1].bus=binds[2].bus=1;
 memcpy(binds[1].rom,buses[1].devices[0].rom.data(),8);memcpy(binds[2].rom,buses[1].devices[1].rom.data(),8);
 assert(tempValidateBindings(binds));memcpy(tempBindings,binds,sizeof binds);tempSyncLogical(nowMs);
 assert(hasTemp(1,nowMs)&&hasTemp(2,nowMs)&&temperature[1]==31&&temperature[2]==42);
 assert(tempBusForLogical(2)==1&&wireError(2,nowMs)=="OK");
 auto saved=binds[2];binds[2]=binds[1];assert(!tempValidateBindings(binds));binds[2]=saved;
 binds[2].bus=0;memcpy(binds[2].rom,buses[0].devices[0].rom.data(),8);assert(!tempValidateBindings(binds));binds[2]=saved;
 TempBinding parsed;assert(tempParseSelection(String("1:")+romHex(binds[1].rom),parsed));assert(parsed.bus==1);
 assert(!tempParseSelection("4:28FF641E23973CAB",parsed));assert(!tempParseSelection("1:28FF641E23973CA0",parsed));assert(tempParseSelection("auto",parsed)&&!parsed.explicitRom);
 TempBindingRecord r={};r.magic=0x42544D50;r.version=1;memcpy(r.bindings,binds,sizeof binds);r.crc=OneWire::crc8((uint8_t*)&r,offsetof(TempBindingRecord,crc));assert(tempBindingRecordValid(r));r.bindings[1].rom[1]^=1;assert(!tempBindingRecordValid(r));
 // A stale cached sample must not be published; bad scratchpad invalidates it.
 assert(!hasTemp(0,nowMs+7000));assert(temperatureInventoryJSON(nowMs+7000).find("STALE_READING")!=std::string::npos);buses[0].devices[0].scratch[8]^=1;run(3000);assert(wireError(0,nowMs)=="SCRATCHPAD_CRC_BAD"&&!hasTemp(0,nowMs));
 buses[0].devices[0]=sensor(1,-12.5);run(3000);assert(hasTemp(0,nowMs)&&temperature[0]==-12.5f);
 // Real 85 C is allowed after a conversion; the documented POR signature is rejected.
 buses[0].devices[0]=sensor(1,85);run(3000);assert(hasTemp(0,nowMs)&&temperature[0]==85);
 buses[0].devices[0].scratch[6]=0x0c;buses[0].devices[0].scratch[8]=OneWire::crc8(buses[0].devices[0].scratch.data(),8);
 run(3000);assert(wireError(0,nowMs)=="POWER_ON_RESET");
 // Disconnected buses recover through periodic discovery; cross-bus ROM aliases fail closed.
 buses[0].devices={sensor(1,30)};buses[2].devices={sensor(1,30)};restart();run(3000);
 assert(wireError(0,nowMs)=="ROM_ON_MULTIPLE_BUSES"&&!hasTemp(0,nowMs));assert(temperatureInventoryJSON(nowMs).find("ROM_ON_MULTIPLE_BUSES")!=std::string::npos);
 // Field case: GPIO17 and GPIO18 both see the same two sensors. Automatic
 // assignment must remain blocked, but explicit UNIQUE ROM+GPIO binding
 // must NOT conceal cross-GPIO observations on unknown wiring.
 buses[2].devices={sensor(2,31),sensor(3,42)};
 buses[1].devices={sensor(2,31),sensor(3,42)};restart();run(3500);
 assert(wireError(1,nowMs)=="ROM_ON_MULTIPLE_BUSES"&&!hasTemp(1,nowMs));
 assert(wireError(2,nowMs)=="ROM_ON_MULTIPLE_BUSES"&&!hasTemp(2,nowMs));
 TempBinding shared[4];for(int i=0;i<4;i++)shared[i]=TempBinding();
 shared[1].explicitRom=shared[2].explicitRom=true;shared[1].bus=1;shared[2].bus=2;
 memcpy(shared[1].rom,buses[1].devices[0].rom.data(),8);
 memcpy(shared[2].rom,buses[2].devices[1].rom.data(),8);
 assert(!tempValidateBindings(shared));
 memcpy(tempBindings,shared,sizeof shared);tempSyncLogical(nowMs);
 assert(!hasTemp(1,nowMs)&&!hasTemp(2,nowMs));
 assert(wireError(1,nowMs)=="ROM_ON_MULTIPLE_BUSES"&&wireError(2,nowMs)=="ROM_ON_MULTIPLE_BUSES");
 assert(temperatureInventoryJSON(nowMs).find("\"shared_line\":true")!=std::string::npos);
 assert(temperatureInventoryJSON(nowMs).find("\"assigned\":true")!=std::string::npos);
 memcpy(shared[2].rom,shared[1].rom,8);
 assert(!tempValidateBindings(shared)); // one sensor cannot fill two named roles
 buses[1].devices.clear();buses[2].devices={sensor(1,30)};
 buses[0].devices={sensor(1,30)};restart();run(3000);
 // No stored explicit selection: ambiguous cross-GPIO reading still blocked.
 assert(wireError(0,nowMs)=="ROM_ON_MULTIPLE_BUSES"&&!hasTemp(0,nowMs));
 buses[2].devices.clear();buses[0].devices[0].external=false;restart();run(3000);
 assert(wireError(0,nowMs)=="PARASITE_POWER_UNSUPPORTED"&&!hasTemp(0,nowMs));
 buses[0].devices[0].external=true;buses[0].low=true;wireProbe[0]=OneWireProbe();tempBus[0]=TempBusState();run(1500);assert(wireError(0,nowMs)=="LINE_LOW");
 buses[0].low=false;run(5000);assert(hasTemp(0,nowMs));
 // CRC-valid all-zero data and out-of-range temperatures are not plausible measurements.
 buses[0].devices[0].scratch.fill(0);run(3000);assert(wireError(0,nowMs)=="SCRATCHPAD_CRC_BAD");
 buses[0].devices[0]=sensor(1,130);run(3000);assert(wireError(0,nowMs)=="OUT_OF_RANGE");
 // At 9-bit resolution, undefined low bits are masked including for negatives.
 buses[0].devices[0]=sensor(1,-12.4375);buses[0].devices[0].scratch[4]=0x1f;
 buses[0].devices[0].scratch[8]=OneWire::crc8(buses[0].devices[0].scratch.data(),8);
 run(3000);assert(hasTemp(0,nowMs)&&temperature[0]==-12.5f);
 // More than four devices cannot silently truncate an inventory and use a broadcast.
 buses[0].devices={sensor(1,20),sensor(5,21),sensor(6,22),sensor(7,23),sensor(8,24)};
 restart();run(3000);assert(wireError(0,nowMs)=="TOO_MANY_ROMS"&&!hasTemp(0,nowMs));
 buses[0].devices={sensor(1,30)};
 // Explicitly unassigned roles preserve physical diagnostics but never publish a value.
 buses[0].devices={sensor(1,30)};buses[1].devices={sensor(2,31)};buses[2].devices={sensor(3,32)};restart();run(3000);
 TempBinding unassigned;assert(tempParseSelection("none",unassigned));assert(unassigned.bus==255&&!unassigned.explicitRom);
 tempBindings[1]=unassigned;tempSyncLogical(nowMs);
 assert(wireError(1,nowMs)=="UNASSIGNED"&&!hasTemp(1,nowMs)&&!isfinite(temperature[1]));
 assert(temperatureDiagnosticsJSON(nowMs).find("\"unassigned\":true")!=std::string::npos);
 assert(tempValidateBindings(tempBindings));
 TempBindingRecord noneRecord={};noneRecord.magic=0x42544D50;noneRecord.version=2;
 memcpy(noneRecord.bindings,tempBindings,sizeof(tempBindings));noneRecord.crc=OneWire::crc8((uint8_t*)&noneRecord,offsetof(TempBindingRecord,crc));
 assert(tempBindingRecordValid(noneRecord));noneRecord.bindings[1].bus=99;
 noneRecord.crc=OneWire::crc8((uint8_t*)&noneRecord,offsetof(TempBindingRecord,crc));assert(!tempBindingRecordValid(noneRecord));
 // GPIO17/18 LOW takes precedence even if a stale inventory claims presence.
 for(int b:{1,2}){
   tempBindings[b]=TempBinding();int old=wireProbe[b].idleLevel;wireProbe[b].idleLevel=0;
   assert(!hasTemp(b,nowMs)&&wireError(b,nowMs)=="LINE_LOW");wireProbe[b].idleLevel=old;
   old=wireProbe[b].afterResetLevel;wireProbe[b].afterResetLevel=0;
   assert(!hasTemp(b,nowMs)&&wireError(b,nowMs)=="LINE_LOW");wireProbe[b].afterResetLevel=old;
 }
 // A corrupted-but-CRC-valid binding record must not let one physical ROM fill two roles.
 tempBindings[2].explicitRom=true;tempBindings[2].bus=0;memcpy(tempBindings[2].rom,buses[0].devices[0].rom.data(),8);
 assert(!hasTemp(0,nowMs)&&!hasTemp(2,nowMs));
 assert(wireError(2,nowMs)=="ROM_ASSIGNED_MULTIPLE_ROLES");
 // Missing bound ROM is different from an intentionally unassigned role.
 tempBindings[2].bus=2;memcpy(tempBindings[2].rom,buses[3].devices[0].rom.data(),8);
 assert(wireError(2,nowMs)=="BOUND_ROM_MISSING");
 tempBindings[2]=TempBinding();
 // No silent substitution for failed ROM CRC, parasite power or a stopped conversion.
 buses[2].devices[0].rom[7]^=1;restart();run(3000);
 assert(!hasTemp(2,nowMs)&&wireError(2,nowMs)=="ROM_CRC_BAD");
 buses[2].devices[0]=sensor(3,32);restart();run(3000);
 assert(wireError(2,nowMs+TEMP_STALE_MS)=="STALE_READING");
 // A stuck conversion cannot turn an old, CRC-valid scratchpad into a fresh sample.
 buses[2].conversionBusy=true;tempBus[2].phase=TempPhase::Converting;tempBus[2].convertedAt=nowMs-TEMP_CONVERSION_MS;tempNextBus=2;
 temperatureTick(nowMs);assert(!hasTemp(2,nowMs)&&wireError(2,nowMs)=="CONVERSION_TIMEOUT");
 buses[2].conversionBusy=false;run(3000);assert(hasTemp(2,nowMs));
 // Field condition reproduced on the actual simulated GPIO17 and GPIO18 buses.
 restart();buses[1].low=true;buses[2].low=true;run(2200);
 for(int b:{1,2}){assert(wireProbe[b].idleLevel==0&&wireProbe[b].afterResetLevel==0&&!wireProbe[b].presence);assert(wireError(b,nowMs)=="LINE_LOW"&&!hasTemp(b,nowMs));}
 assert(hasTemp(0,nowMs)&&hasTemp(3,nowMs));
 tempBindings[1]=unassigned;tempSyncLogical(nowMs);
 assert(wireError(1,nowMs)=="UNASSIGNED");
 assert(temperatureDiagnosticsJSON(nowMs).find("\"electrical_state\":\"LINE_LOW\"")!=std::string::npos);
 // A CRC-valid foreign family and repeat ROM during search have separate reasons.
 restart();buses[2].devices[0]=sensor(3,32);buses[2].devices[0].rom[0]=0x10;
 buses[2].devices[0].rom[7]=OneWire::crc8(buses[2].devices[0].rom.data(),7);run(2500);
 assert(wireError(2,nowMs)=="NOT_DS18B20"&&!hasTemp(2,nowMs));
 buses[2].devices={sensor(3,32),sensor(3,32)};restart();run(2500);
 assert(wireError(2,nowMs)=="ROM_SEARCH_NOISE"&&!hasTemp(2,nowMs));
 buses[2].devices={sensor(3,32)};
 // Persisted binding corruption fails closed, rather than silently guessing roles.
 tempLoadBindingRecord(nullptr,true);assert(tempBindingLoadInvalid);
 for(int i=0;i<4;i++)assert(tempIsUnassigned(i)&&!hasTemp(i,nowMs));
 tempLoadBindingRecord(nullptr,false);assert(!tempBindingLoadInvalid&&!tempIsUnassigned(0));
 TempBindingRecord loadRecord={};loadRecord.magic=0x42544D50U;loadRecord.version=2;
 for(auto& binding:loadRecord.bindings)binding.bus=255;
 loadRecord.crc=OneWire::crc8((uint8_t*)&loadRecord,offsetof(TempBindingRecord,crc));
 tempLoadBindingRecord(&loadRecord,true);assert(!tempBindingLoadInvalid&&tempIsUnassigned(0));
 loadRecord.crc^=1;tempLoadBindingRecord(&loadRecord,true);assert(tempBindingLoadInvalid&&tempIsUnassigned(0));
 tempLoadBindingRecord(nullptr,false);
 // Millis rollover retains the conversion deadline and sample validity.
 nowMs=UINT32_MAX-500;restart();run(2500);assert(hasTemp(0,nowMs));
 std::cout<<"PASS: shipping DS18B20 engine: timing, CRC, two ROM binding, duplicates, NVS CRC, stale/POR, missing/low line, recovery, rollover"<<std::endl;
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'temp.cpp';p.write_text(stub+'\n#include "BUSA_Temperature.h"\n'+tests)
 exe=Path(d)/'temp';subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
