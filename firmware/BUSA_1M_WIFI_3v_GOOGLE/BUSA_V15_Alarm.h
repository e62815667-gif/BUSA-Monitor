#pragma once
// BUSA-1M #001 K1..K4: ONLY 12 V sound annunciators, NOT process equipment.
// 3.3 V ESP32 GPIO logic, electrical relay driver interface must accept 3.3 V.
// Relay board is assumed ACTIVE LOW; adjust this constant for an active-HIGH model.
#include "BUSA_V15_AlarmLogic.h"
extern volatile bool googleRestart; // declared in BUSA_Google.h later in the module chain
constexpr bool BUSA_ALARM_ACTIVE_LOW=true;
constexpr uint32_t BUSA_ALARM_POLL_MS=200U;
BusaAlarm::Channel alarmChannels[4];
bool alarmEnabled[4]={false,false,false,false}; // never inherit legacy display settings
bool alarmGpioInitialized=false;
uint32_t alarmLastPoll=0;
int alarmPhysicalLevel(bool on){
  return BUSA_ALARM_ACTIVE_LOW?(on?LOW:HIGH):(on?HIGH:LOW);
}
void alarmOutputsOff(){
  for(int i=0;i<4;i++){
    if(alarmGpioInitialized)digitalWrite(KPIN[i],alarmPhysicalLevel(false));
    alarmChannels[i].on=false;
  }
}
void alarmInitPins(){
  // preload inactive output latch before switching GPIO direction.
  for(int i=0;i<4;i++){
    digitalWrite(KPIN[i],alarmPhysicalLevel(false));
    pinMode(KPIN[i],OUTPUT);
  }
  alarmGpioInitialized=true;
  alarmOutputsOff();
}
void alarmLoadSettings(){
  for(int i=0;i<4;i++)alarmEnabled[i]=prefs.getBool((String("aen")+i).c_str(),false);
}
void alarmPoll(uint32_t now){
  if(!alarmGpioInitialized)return;
  if(alarmLastPoll && !elapsed(now,alarmLastPoll,BUSA_ALARM_POLL_MS))return;
  alarmLastPoll=now;
  if(otaActive || otaRestart || googleRestart || tftRebootPending){
    alarmOutputsOff();
    return;
  }
  for(int i=0;i<4;i++){
    const bool fresh=hasTemp(i,now);
    const float c=fresh?temperature[i]:NAN;
    const bool sound=BusaAlarm::evaluate(alarmChannels[i],alarmEnabled[i],fresh,c,refTemp[i],noticeTemp[i]);
    const int level=alarmPhysicalLevel(sound);
    digitalWrite(KPIN[i],level);
  }
}
String alarmStatusJSON(uint32_t now){
  String s; s.reserve(1050);
  s="{\"purpose\":\"12V_SOUND_ALERTS_ONLY\",\"driver_enabled\":true,";
  s+="\"trigger_level\":\""+String(BUSA_ALARM_ACTIVE_LOW?"LOW":"HIGH")+"\",";
  s+="\"channels\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"channel\":"+String(i+1)+",\"gpio\":"+String(KPIN[i]);
    s+=",\"enabled\":"+(alarmEnabled[i]?String("true"):String("false"));
    s+=",\"on_c\":"+(isfinite(refTemp[i])?String(refTemp[i],1):String("null"));
    s+=",\"off_c\":"+(isfinite(noticeTemp[i])?String(noticeTemp[i],1):String("null"));
    s+=",\"sample_valid\":"+(hasTemp(i,now)?String("true"):String("false"));
    s+=",\"sound_on\":"+(alarmChannels[i].on?String("true"):String("false"));
    s+=",\"reason\":\""+String(BusaAlarm::reasonText(alarmChannels[i].reason))+"\"}";
  }
  return s+"]}";
}
