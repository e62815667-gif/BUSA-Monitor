#pragma once
// GPIO7,6,5,4: low-voltage relay module for 12V SOUND annunciators ONLY.
// Driver INPUT polarity is assumed active LOW; verify electrical interface.
// Never connect power equipment to these GPIOs.
#include "BUSA_SoundAlarmLogic.h"
constexpr bool BUSA_SOUND_ACTIVE_LOW=true;
constexpr uint32_t BUSA_SOUND_POLL_MS=200U;
BusaSound::Channel soundChannels[4];
bool soundEnabled[4]={false,false,false,false};
float soundOnC[4]={NAN,NAN,NAN,NAN},soundOffC[4]={NAN,NAN,NAN,NAN};
bool soundGpioInitialized=false;
uint32_t soundLastPoll=0;
int soundPinLevel(bool on){return BUSA_SOUND_ACTIVE_LOW?(on?LOW:HIGH):(on?HIGH:LOW);}
void soundOutputsOff(){
  for(int i=0;i<4;i++){
    if(soundGpioInitialized)digitalWrite(KPIN[i],soundPinLevel(false));
    soundChannels[i].on=false;
  }
}
void soundInitPins(){
  // Preload OFF level before enabling physical pin output.
  for(int i=0;i<4;i++){digitalWrite(KPIN[i],soundPinLevel(false));pinMode(KPIN[i],OUTPUT);}
  soundGpioInitialized=true;
  soundOutputsOff();
}
void soundLoadPrefs(){
  // Do not import V14 rf/nt or V15 aen: these could have incompatible meaning.
  for(int i=0;i<4;i++){
    String k=String("sndE")+String(i);
    soundOnC[i]=prefs.getFloat((String("sndOn")+String(i)).c_str(),NAN);
    soundOffC[i]=prefs.getFloat((String("sndOff")+String(i)).c_str(),NAN);
    soundEnabled[i]=prefs.getBool(k.c_str(),false) && BusaSound::valid(soundOnC[i],soundOffC[i]);
    soundChannels[i].on=false;
  }
}
void soundPoll(uint32_t now){
  if(!soundGpioInitialized)return;
  if(soundLastPoll && (uint32_t)(now-soundLastPoll)<BUSA_SOUND_POLL_MS)return;
  soundLastPoll=now;
  if(otaActive||otaRestart||tftRebootPending){soundOutputsOff();return;}
  for(int i=0;i<4;i++){
    bool valid=hasTemp(i,now);
    const bool on=BusaSound::evaluate(soundChannels[i],soundEnabled[i],
      valid,valid?temperature[i]:NAN,soundOnC[i],soundOffC[i]);
    digitalWrite(KPIN[i],soundPinLevel(on));
  }
}
String soundJSON(uint32_t now){
  String s="{\"purpose\":\"12V_SOUND_ONLY\",\"active_low\":";
  s+=(BUSA_SOUND_ACTIVE_LOW?String("true"):String("false"));
  s+=",\"channels\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"channel\":"+String(i+1)+",\"gpio\":"+String(KPIN[i]);
    s+=",\"enabled\":"+(soundEnabled[i]?String("true"):String("false"));
    s+=",\"on_c\":"+(std::isfinite(soundOnC[i])?String(soundOnC[i],1):String("null"));
    s+=",\"off_c\":"+(std::isfinite(soundOffC[i])?String(soundOffC[i],1):String("null"));
    s+=",\"sound_on\":"+(soundChannels[i].on?String("true"):String("false"));
    s+=",\"sample_valid\":"+(hasTemp(i,now)?String("true"):String("false"));
    s+=",\"reason\":\""+String(BusaSound::reasonText(soundChannels[i].reason))+"\"}";
  }
  return s+"]}";
}
