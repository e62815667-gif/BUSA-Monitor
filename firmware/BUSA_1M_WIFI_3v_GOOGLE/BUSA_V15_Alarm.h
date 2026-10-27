#pragma once
// Hardware polarity and interface are unknown. This release has NO output driver.
// INPUT/no pull avoids actively sourcing/sinking K1..K4; it cannot guarantee a
// floating external module is off. Disconnect driver power until wiring review.
#include "BUSA_V15_AlarmLogic.h"
constexpr uint32_t BUSA_ALARM_POLL_MS=200U;
BusaAlarm::Channel alarmChannels[4];
bool alarmEnabled[4]={false,false,false,false};
bool alarmGpioInitialized=false;
uint32_t alarmLastPoll=0;
void alarmOutputsOff(){
  for(int i=0;i<4;i++){
    alarmEnabled[i]=false;alarmChannels[i].on=false;
    alarmChannels[i].reason=BusaAlarm::HardwareLocked;
  }
}
void alarmInitPins(){
  for(int i=0;i<4;i++)pinMode(KPIN[i],INPUT); // Arduino INPUT disables internal pulls.
  alarmGpioInitialized=false; // No physical output driver is initialized.
  alarmOutputsOff();
}
void alarmLoadSettings(){
  // Old aen0..aen3 in NVS never arm a replacement board. Leave stored values untouched.
  alarmOutputsOff();
}
void alarmPoll(uint32_t now){
  (void)now;
  // Also safe during boot, OTA, reset requests and deliberate HTTP re-entry.
  alarmOutputsOff();
}
String alarmStatusJSON(uint32_t now){
  String s; s.reserve(1050);
  s="{\"purpose\":\"MONITOR_ONLY_HARDWARE_LOCK\",\"driver_enabled\":false,";
  s+="\"hardware_approved\":false,\"gpio_mode\":\"INPUT_NO_PULL\",\"trigger_level\":null,";
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
