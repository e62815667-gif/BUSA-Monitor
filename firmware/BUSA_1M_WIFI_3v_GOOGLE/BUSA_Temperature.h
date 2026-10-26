#pragma once
#include <cstring>
#include <cstddef>
// DS18B20 protocol: externally powered sensors only. One small bus operation
// per loop; conversion waits use millis, never delay or EEPROM writes.
constexpr uint8_t TEMP_MAX_DEVICES=4;
constexpr uint32_t TEMP_CONVERSION_MS=825U,TEMP_CYCLE_MS=2500U,TEMP_PROBE_MS=12000U,TEMP_STALE_MS=7000U;
struct TempDevice {
  uint8_t rom[8]={0};bool external=false,hasSample=false;
  float value=NAN;uint32_t sampledAt=0;const char* error="WAIT_CONVERSION";
};
struct OneWireProbe {
  bool checked=false,presence=false,romFound=false,crcOk=false,familyOk=false,multiple=false,searchNoise=false,secondFound=false,overflow=false;
  int8_t idleLevel=-1,afterResetLevel=-1;
  uint8_t rom[8]={0},secondRom[8]={0},count=0;
  uint32_t checkedAt=0;
  TempDevice devices[TEMP_MAX_DEVICES];
};
enum class TempPhase:uint8_t {Probe,Search,Power,Start,Converting,Read,Rest};
struct TempBusState {TempPhase phase=TempPhase::Probe;uint8_t cursor=0;uint32_t convertedAt=0,cycleAt=0;OneWireProbe pending;};
OneWireProbe wireProbe[4];TempBusState tempBus[4];
uint32_t oneWireNoPresenceCount[4]={0},oneWireDetectedCount[4]={0};
float temperature[4]={NAN,NAN,NAN,NAN};bool tempPresent[4]={false};uint32_t tempAt[4]={0};
struct TempBinding {uint8_t bus=0,rom[8]={0};bool explicitRom=false;};
TempBinding tempBindings[4];
uint8_t tempNextBus=0;
String romHex(const uint8_t rom[8]){char out[17];for(int i=0;i<8;i++)snprintf(out+i*2,3,"%02X",rom[i]);return String(out);}
bool tempRomValid(const uint8_t* rom){return rom[0]==0x28 && OneWire::crc8(rom,7)==rom[7];}
bool tempProbeClean(const OneWireProbe& p){return p.presence && p.count && !p.searchNoise && !p.overflow && p.familyOk && p.crcOk;}
int tempBusForLogical(int n){return tempBindings[n].explicitRom?tempBindings[n].bus:n;}
int tempDeviceForLogical(int n){
  if(n<0||n>3)return -1;
  const auto& bind=tempBindings[n];const auto& p=wireProbe[tempBusForLogical(n)];
  if(bind.explicitRom){for(int k=0;k<p.count;k++)if(memcmp(bind.rom,p.devices[k].rom,8)==0)return k;return -1;}
  return p.count==1?0:-1;
}
bool tempRomUniqueAcrossBuses(const uint8_t* rom){
  int count=0;for(const auto& p:wireProbe)for(int k=0;k<p.count;k++)if(memcmp(p.devices[k].rom,rom,8)==0)++count;
  return count==1;
}
bool oneWireVerified(int n){
  if(n<0||n>3)return false;
  int b=tempBusForLogical(n),k=tempDeviceForLogical(n);
  return k>=0 && tempProbeClean(wireProbe[b]) && tempRomUniqueAcrossBuses(wireProbe[b].devices[k].rom);
}
bool hasTemp(int n,uint32_t now){
  if(!oneWireVerified(n))return false;
  const auto& d=wireProbe[tempBusForLogical(n)].devices[tempDeviceForLogical(n)];
  return d.external && d.hasSample && !elapsed(now,d.sampledAt,TEMP_STALE_MS) && isfinite(d.value);
}
String wireError(int n,uint32_t now){
  int b=tempBusForLogical(n),k=tempDeviceForLogical(n);const auto& p=wireProbe[b];
  if(!p.checked)return "WAIT_DIAG";
  if(!p.presence)return p.idleLevel==0?"LINE_LOW":"NO_PRESENCE";
  if(p.overflow)return "TOO_MANY_ROMS";
  if(p.searchNoise)return "ROM_SEARCH_NOISE";
  if(!p.familyOk && p.romFound)return "NOT_DS18B20";
  if(!p.crcOk && p.romFound)return "ROM_CRC_BAD";
  if(!p.count)return "NO_ROM";
  if(k<0)return tempBindings[n].explicitRom?"BOUND_ROM_MISSING":"MULTIPLE_ROM";
  const auto& d=p.devices[k];
  if(!tempRomUniqueAcrossBuses(d.rom))return "ROM_ON_MULTIPLE_BUSES";
  if(!d.external)return String(d.error);
  if(!d.hasSample)return String(d.error);
  return hasTemp(n,now)?"OK":"STALE_READING";
}
void tempSyncLogical(uint32_t now){
  for(int i=0;i<4;i++){
    tempPresent[i]=tempDeviceForLogical(i)>=0 && wireProbe[tempBusForLogical(i)].presence;
    if(hasTemp(i,now)){const auto& d=wireProbe[tempBusForLogical(i)].devices[tempDeviceForLogical(i)];temperature[i]=d.value;tempAt[i]=d.sampledAt;}
    else{temperature[i]=NAN;tempAt[i]=0;}
  }
}
void tempInvalidateBus(uint8_t b,const char* error){
  for(int k=0;k<wireProbe[b].count;k++){auto& d=wireProbe[b].devices[k];d.hasSample=false;d.value=NAN;d.error=error;}
}
void tempCommitProbe(uint8_t b,uint32_t now){
  auto& state=tempBus[b];const OneWireProbe old=wireProbe[b];
  state.pending.checked=true;state.pending.checkedAt=now;wireProbe[b]=state.pending;
  // Keep only still-identical, recent physical readings through routine discovery.
  for(int k=0;k<wireProbe[b].count;k++)for(int j=0;j<old.count;j++){
    if(memcmp(wireProbe[b].devices[k].rom,old.devices[j].rom,8)==0){
      wireProbe[b].devices[k]=old.devices[j];break;
    }
  }
  auto& p=wireProbe[b];p.romFound=p.count>0;
  if(p.count){memcpy(p.rom,p.devices[0].rom,8);p.crcOk=!p.searchNoise;p.familyOk=p.familyOk && tempRomValid(p.rom);}
  if(p.count>1){p.secondFound=p.multiple=true;memcpy(p.secondRom,p.devices[1].rom,8);}
  state.cursor=0;state.phase=tempProbeClean(p)?TempPhase::Power:TempPhase::Rest;
  state.cycleAt=now;
}
void temperatureTick(uint32_t now){
  const uint8_t b=tempNextBus;tempNextBus=(tempNextBus+1)%4;
  auto& st=tempBus[b];auto& p=wireProbe[b];OneWire* bus=temperatureBuses[b];
  if(st.phase==TempPhase::Probe){
    st.pending=OneWireProbe();st.pending.familyOk=true;st.pending.crcOk=true;
    st.pending.idleLevel=digitalRead(TPIN[b]);st.pending.presence=bus->reset()!=0;
    st.pending.afterResetLevel=digitalRead(TPIN[b]);bus->reset_search();
    if(st.pending.presence){++oneWireDetectedCount[b];st.phase=TempPhase::Search;}
    else{++oneWireNoPresenceCount[b];tempCommitProbe(b,now);}
  }else if(st.phase==TempPhase::Search){
    uint8_t rom[8]={0};
    if(!bus->search(rom)){bus->reset_search();tempCommitProbe(b,now);}
    else if(OneWire::crc8(rom,7)!=rom[7]){st.pending.searchNoise=true;tempCommitProbe(b,now);}
    else if(rom[0]!=0x28){st.pending.familyOk=false;st.pending.searchNoise=true;tempCommitProbe(b,now);}
    else if(st.pending.count>=TEMP_MAX_DEVICES){st.pending.overflow=true;tempCommitProbe(b,now);}
    else{
      bool duplicate=false;for(int k=0;k<st.pending.count;k++)if(memcmp(rom,st.pending.devices[k].rom,8)==0)duplicate=true;
      if(duplicate){st.pending.searchNoise=true;tempCommitProbe(b,now);}
      else memcpy(st.pending.devices[st.pending.count++].rom,rom,8);
    }
  }else if(st.phase==TempPhase::Power){
    auto& d=p.devices[st.cursor];
    if(bus->reset()){
      bus->select(d.rom);bus->write(0xB4);d.external=bus->read_bit()!=0;bus->reset();
      d.error=d.external?"WAIT_CONVERSION":"PARASITE_POWER_UNSUPPORTED";
    }else{d.external=false;d.error="NO_PRESENCE";}
    if(++st.cursor>=p.count){st.cursor=0;st.phase=TempPhase::Start;}
  }else if(st.phase==TempPhase::Start){
    bool external=true;for(int k=0;k<p.count;k++)external=external&&p.devices[k].external;
    if(!tempProbeClean(p)||!external){
      if(!external)for(int k=0;k<p.count;k++)if(p.devices[k].external){p.devices[k].hasSample=false;p.devices[k].error="BUS_POWER_UNSUPPORTED";}
      st.phase=TempPhase::Rest;st.cycleAt=now;
    }
    else if(!bus->reset()){tempInvalidateBus(b,"NO_PRESENCE");st.phase=TempPhase::Probe;}
    else{
      // Broadcast only after the complete inventory contained DS18B20s only.
      bus->skip();bus->write(0x44,0);st.convertedAt=millis();st.cycleAt=st.convertedAt;
      st.phase=TempPhase::Converting;
    }
  }else if(st.phase==TempPhase::Converting){
    if(elapsed(now,st.convertedAt,TEMP_CONVERSION_MS)){st.cursor=0;st.phase=TempPhase::Read;}
  }else if(st.phase==TempPhase::Read){
    auto& d=p.devices[st.cursor];uint8_t scratch[9]={0};d.hasSample=false;d.value=NAN;
    if(!bus->reset())d.error="NO_PRESENCE";
    else{
      bus->select(d.rom);bus->write(0xBE);for(uint8_t j=0;j<9;j++)scratch[j]=bus->read();bus->reset();
      bool zeros=true;for(uint8_t j=0;j<9;j++)zeros=zeros&&scratch[j]==0;
      if(zeros || OneWire::crc8(scratch,8)!=scratch[8])d.error="SCRATCHPAD_CRC_BAD";
      else if(scratch[0]==0x50 && scratch[1]==0x05 && scratch[6]==0x0C)d.error="POWER_ON_RESET";
      else if(scratch[0]==0xFF && scratch[1]==0x07)d.error="INSUFFICIENT_POWER";
      else if((scratch[4]&0x9F)!=0x1F)d.error="CONFIG_INVALID";
      else{
        int16_t raw=(int16_t)(((uint16_t)scratch[1]<<8)|scratch[0]);
        const uint8_t resolution=(scratch[4]>>5)&3;
        if(resolution<3)raw=(int16_t)(raw & ~((1<<(3-resolution))-1));
        const float value=(float)raw/16.0f;
        if(!isfinite(value)||value< -55.0f||value>125.0f)d.error="OUT_OF_RANGE";
        else{d.value=value;d.hasSample=true;d.sampledAt=millis();d.error="OK";}
      }
    }
    if(++st.cursor>=p.count){st.cursor=0;st.phase=TempPhase::Rest;}
  }else if(st.phase==TempPhase::Rest){
    if(elapsed(now,p.checkedAt,TEMP_PROBE_MS) || (!p.presence && elapsed(now,p.checkedAt,2000U)))st.phase=TempPhase::Probe;
    else if(elapsed(now,st.cycleAt,TEMP_CYCLE_MS) && tempProbeClean(p))st.phase=TempPhase::Start;
  }
  // A 500 ms UI snapshot period avoids redundant ROM/CRC/memcmp work every loop.
  static uint32_t lastSync=0;
  if(!lastSync || elapsed(now,lastSync,500U)){
    tempSyncLogical(now);
    lastSync=now;
  }
}
// Atomic persisted record: no automatic assignment of two ROMs to named roles.
struct TempBindingRecord {uint32_t magic;uint8_t version,reserved[3];TempBinding bindings[4];uint8_t crc;};
bool tempBindingRecordValid(const TempBindingRecord& r){
  if(r.magic!=0x42544D50U||r.version!=1||OneWire::crc8((const uint8_t*)&r,offsetof(TempBindingRecord,crc))!=r.crc)return false;
  for(const auto& b:r.bindings)if(b.explicitRom && (b.bus>3||!tempRomValid(b.rom)))return false;
  for(int i=0;i<4;i++)for(int j=i+1;j<4;j++)if(r.bindings[i].explicitRom&&r.bindings[j].explicitRom&&memcmp(r.bindings[i].rom,r.bindings[j].rom,8)==0)return false;
  return true;
}
bool tempValidateBindings(const TempBinding candidate[4]){
  for(int i=0;i<4;i++){
    const auto& bind=candidate[i];const uint8_t b=bind.explicitRom?bind.bus:i;if(b>3)return false;
    const auto& p=wireProbe[b];int found=-1;
    if(bind.explicitRom){
      if(!tempRomValid(bind.rom)||!tempProbeClean(p))return false;
      for(int k=0;k<p.count;k++)if(memcmp(bind.rom,p.devices[k].rom,8)==0)found=k;
      if(found<0||!tempRomUniqueAcrossBuses(bind.rom))return false;
    }else if(p.count==1)found=0;
    if(found>=0){
      for(int j=0;j<i;j++){
        const auto& other=candidate[j];const auto& q=wireProbe[other.explicitRom?other.bus:j];
        const uint8_t* rom=other.explicitRom?other.rom:(q.count==1?q.devices[0].rom:nullptr);
        if(rom && (bind.explicitRom||other.explicitRom) && memcmp(rom,p.devices[found].rom,8)==0)return false;
      }
    }
  }
  return true;
}
bool tempParseSelection(const String& selection,TempBinding& b){
  b=TempBinding();if(selection=="auto")return true;
  if(selection.length()!=18 || selection[0]<'0'||selection[0]>'3'||selection[1]!=':')return false;
  b.bus=(uint8_t)(selection[0]-'0');b.explicitRom=true;
  for(int i=0;i<8;i++){
    uint8_t v=0;for(int j=0;j<2;j++){const char c=selection[2+i*2+j];int n=c>='0'&&c<='9'?c-'0':c>='A'&&c<='F'?c-'A'+10:c>='a'&&c<='f'?c-'a'+10:-1;if(n<0)return false;v=(v<<4)|(uint8_t)n;}b.rom[i]=v;
  }
  return tempRomValid(b.rom);
}
String temperatureInventoryJSON(uint32_t now){
  String s;s.reserve(2300);s="[";
  for(int b=0;b<4;b++)for(int k=0;k<wireProbe[b].count;k++){
    const auto& d=wireProbe[b].devices[k];if(s.length()>1)s+=",";
    const bool valid=tempProbeClean(wireProbe[b])&&tempRomUniqueAcrossBuses(d.rom)&&d.hasSample&&d.external&&!elapsed(now,d.sampledAt,TEMP_STALE_MS);
    s+="{\"bus\":"+String(b)+",\"gpio\":"+String(TPIN[b])+",\"rom\":"+quoteJson(romHex(d.rom));
    s+=",\"verified\":"+String(valid?"true":"false")+",\"c\":"+(valid?String(d.value,2):String("null"));
    String error=valid?String("OK"):String(d.error);
    if(wireProbe[b].overflow)error="TOO_MANY_ROMS";
    else if(wireProbe[b].searchNoise)error="ROM_SEARCH_NOISE";
    else if(!tempRomUniqueAcrossBuses(d.rom))error="ROM_ON_MULTIPLE_BUSES";
    else if(d.hasSample && elapsed(now,d.sampledAt,TEMP_STALE_MS))error="STALE_READING";
    s+=",\"error\":"+quoteJson(error)+"}";
  }
  return s+"]";
}
String temperatureDiagnosticsJSON(uint32_t now){
  String s;s.reserve(1800);s="[";
  for(int i=0;i<4;i++){
    const int b=tempBusForLogical(i),k=tempDeviceForLogical(i);const auto& p=wireProbe[b];if(i)s+=",";
    s+="{\"gpio\":"+String(TPIN[b])+",\"probe_ms\":"+String(p.checkedAt);
    s+=",\"presence\":"+String(tempPresent[i]?"true":"false")+",\"measurement_verified\":"+String(hasTemp(i,now)?"true":"false");
    s+=",\"no_presence_count\":"+String(oneWireNoPresenceCount[b])+",\"detected_count\":"+String(oneWireDetectedCount[b]);
    s+=",\"rom_found\":"+String(k>=0?"true":"false")+",\"rom_crc_ok\":"+String(oneWireVerified(i)?"true":"false");
    s+=",\"family_ds18b20\":"+String(p.familyOk?"true":"false")+",\"multiple_rom\":"+String(p.multiple?"true":"false");
    s+=",\"idle_gpio\":"+String(p.idleLevel)+",\"after_reset_gpio\":"+String(p.afterResetLevel);
    s+=",\"rom\":"+quoteJson(k>=0?romHex(p.devices[k].rom):String(""));
    s+=",\"second_rom\":"+quoteJson(p.secondFound?romHex(p.secondRom):String(""));
    s+=",\"rom_search_noise\":"+String(p.searchNoise?"true":"false");
    s+=",\"binding_confirmed\":"+String(tempBindings[i].explicitRom?"true":"false")+",\"result\":"+quoteJson(wireError(i,now))+"}";
  }
  return s+"]";
}
