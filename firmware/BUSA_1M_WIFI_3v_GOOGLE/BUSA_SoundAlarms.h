#pragma once
// BUSA-1M: four 12 V SOUND-ONLY relay channels. No actuator or power control.
// K1: normal CUBE temperature condition, continuous audible indication.
// K2: "valve closed" indication for 90 seconds per T_ON event.
// K3: "valve open" indication for 90 seconds per T_ON event.
// K4: emergency condition SOUND indication, continuous while active.
// All 4 have configurable temperature sensor, T_ON, T_OFF and enable.
#include "BUSA_SoundAlarmLogic.h"
constexpr bool BUSA_SOUND_ACTIVE_LOW=true;
constexpr uint32_t BUSA_SOUND_POLL_MS=200U;
constexpr uint32_t BUSA_SOUND_TIMED_MS=90000UL;
BusaSound::Channel soundChannels[4],soundThreshold[4];
BusaSound::TimedSignal soundTimed[4];
bool soundEnabled[4]={true,true,true,true};
uint8_t soundSensorIndex[4]={2,1,1,3}; // T3 cube; T2 column; T2 column; T4 top
float soundOnC[4]={NAN,NAN,NAN,NAN},soundOffC[4]={NAN,NAN,NAN,NAN};
bool soundGpioInitialized=false;
uint32_t soundLastPoll=0,soundSignalCount[4]={0},soundLastStartMs[4]={0};
int soundPinLevel(bool on){return BUSA_SOUND_ACTIVE_LOW?(on?LOW:HIGH):(on?HIGH:LOW);}
const char* soundRoleName(uint8_t i){
  const char* names[]={"CUBE_NORMAL_SOUND","VALVE_CLOSED_SOUND_90S",
    "VALVE_OPEN_SOUND_90S","EMERGENCY_SOUND_ONLY"};
  return i<4?names[i]:"UNKNOWN";
}
bool soundIsTimed(uint8_t i){return i==1||i==2;}
void soundOutputsOff(){
  for(int i=0;i<4;i++){
    if(soundGpioInitialized)digitalWrite(KPIN[i],soundPinLevel(false));
    soundChannels[i].on=false;
  }
}
void soundInitPins(){
  // GPIOs are driven only to the sound relay board. Active-low outputs.
  for(int i=0;i<4;i++){digitalWrite(KPIN[i],soundPinLevel(false));pinMode(KPIN[i],OUTPUT);}
  soundGpioInitialized=true;
  soundOutputsOff();
}
void soundResetState(){
  for(int i=0;i<4;i++){
    soundChannels[i]=BusaSound::Channel();
    soundThreshold[i]=BusaSound::Channel();
    BusaSound::resetTimed(soundTimed[i]);
  }
  soundLastPoll=0;
}
void soundLoadPrefs(){
  // One-time migration: enable the four MONITORING signals as requested.
  // No physical sound until the user configures valid T_ON/T_OFF values.
  // Thereafter individual on/off settings persist through restarts.
  if(!prefs.getBool("sndRolesV1",false)){
    for(int i=0;i<4;i++){
      const String key=String("sndE")+i;
      prefs.putBool(key.c_str(),true);
    }
    prefs.putBool("sndRolesV1",true);
  }
  const uint8_t defaults[4]={2,1,1,3};
  for(int i=0;i<4;i++){
    String k=String("sndE")+i;
    soundOnC[i]=prefs.getFloat((String("sndOn")+i).c_str(),NAN);
    soundOffC[i]=prefs.getFloat((String("sndOff")+i).c_str(),NAN);
    soundEnabled[i]=prefs.getBool(k.c_str(),true);
    soundSensorIndex[i]=prefs.getUChar((String("sndS")+i).c_str(),defaults[i]);
    if(soundSensorIndex[i]>3)soundSensorIndex[i]=defaults[i];
  }
  soundResetState();
}
void soundPoll(uint32_t now){
  if(!soundGpioInitialized)return;
  if(soundLastPoll && (uint32_t)(now-soundLastPoll)<BUSA_SOUND_POLL_MS)return;
  soundLastPoll=now;
  if(otaActive||otaRestart||tftRebootPending){soundOutputsOff();return;}
  for(int i=0;i<4;i++){
    const bool oldSound=soundChannels[i].on;
    const int sensor=soundSensorIndex[i];
    const bool fresh=hasTemp(sensor,now);
    const float temp=fresh?temperature[sensor]:NAN;
    bool on=false;
    if(soundIsTimed(i)){
      on=BusaSound::evaluateTimed(soundThreshold[i],soundTimed[i],soundEnabled[i],
          fresh,temp,soundOnC[i],soundOffC[i],now,BUSA_SOUND_TIMED_MS);
      soundChannels[i].on=on;
      soundChannels[i].reason=on?BusaSound::Sounding:soundThreshold[i].reason;
    }else{
      on=BusaSound::evaluate(soundChannels[i],soundEnabled[i],
          fresh,temp,soundOnC[i],soundOffC[i]);
    }
    if(on&&!oldSound){soundLastStartMs[i]=now;++soundSignalCount[i];}
    digitalWrite(KPIN[i],soundPinLevel(on));
  }
}
String soundJSON(uint32_t now){
  String s="{\"purpose\":\"12V_SOUND_ONLY\",\"active_low\":";
  s+=(BUSA_SOUND_ACTIVE_LOW?String("true"):String("false"));
  s+=",\"control_of_valve_or_power\":false";
  s+=",\"channels\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    const uint8_t sensor=soundSensorIndex[i];
    const bool timed=soundIsTimed(i);
    const uint32_t remaining=timed?BusaSound::remainingMs(soundTimed[i],now,BUSA_SOUND_TIMED_MS):0;
    s+="{\"channel\":"+String(i+1)+",\"gpio\":"+String(KPIN[i]);
    s+=",\"role\":\""+String(soundRoleName(i))+"\"";
    s+=",\"sensor_index\":"+String(sensor);
    s+=",\"temperature_gpio\":"+String(TPIN[sensor]);
    s+=",\"enabled\":"+(soundEnabled[i]?String("true"):String("false"));
    s+=",\"on_c\":"+(std::isfinite(soundOnC[i])?String(soundOnC[i],1):String("null"));
    s+=",\"off_c\":"+(std::isfinite(soundOffC[i])?String(soundOffC[i],1):String("null"));
    s+=",\"sound_on\":"+(soundChannels[i].on?String("true"):String("false"));
    s+=",\"sample_valid\":"+(hasTemp(sensor,now)?String("true"):String("false"));
    s+=",\"timed\":"+(timed?String("true"):String("false"));
    s+=",\"duration_seconds\":"+String(timed?90:0);
    s+=",\"remaining_seconds\":"+String((remaining+999UL)/1000UL);
    s+=",\"threshold_active\":"+(timed?(soundThreshold[i].on?String("true"):String("false")):(soundChannels[i].on?String("true"):String("false")));
    s+=",\"rearm_required\":"+(timed&&!soundTimed[i].armed?String("true"):String("false"));
    s+=",\"signal_count\":"+String(soundSignalCount[i]);
    s+=",\"last_signal_start_ms\":"+String(soundLastStartMs[i]);
    s+=",\"command_gpio_level\":"+String(soundPinLevel(soundChannels[i].on));
    s+=",\"physical_relay_feedback\":false";
    s+=",\"reason\":\""+String(BusaSound::reasonText(soundChannels[i].reason))+"\"}";
  }
  return s+"]}";
}
