#pragma once
// BUSA-1M: monitoring and sound alerts ONLY. No valve, power or motor commands.
// T3 (cube) ON: K1 steady sound + K2 90-second closing notification.
// T3 OFF: K3 90-second opening notification; K1 is silenced.
// T4 (top) ON: K4 steady warning + K2 90-second closing notification.
// T4 OFF: K4 off + K3 90-second opening notification; K1 unchanged.
// T3 and T4 each have configurable T_ON/T_OFF thresholds.
#include "BUSA_SoundAlarmLogic.h"
// A separate active-3.3V logic buzzer input on GPIO2. Use a compatible
// low-current active buzzer module or an external driver as required.
constexpr uint8_t BUSA_BUZZER_GPIO=2;
constexpr bool BUSA_BUZZER_ACTIVE_HIGH=true;
BusaSound::Channel buzzerTempChannels[2];
bool buzzerEnabled[2]={true,true};
float buzzerOnC[2]={NAN,NAN},buzzerOffC[2]={NAN,NAN};
bool buzzerSoundOn=false;
uint32_t buzzerLastStartMs=0,buzzerActivationCount=0;
int buzzerPinLevel(bool active){return active==BUSA_BUZZER_ACTIVE_HIGH?HIGH:LOW;}
constexpr bool BUSA_SOUND_ACTIVE_LOW=true;
constexpr uint32_t BUSA_SOUND_POLL_MS=200U;
constexpr uint32_t BUSA_SOUND_TIMED_MS=90000UL;
BusaSound::Routed soundRoute;
BusaSound::Channel soundChannels[4];
bool soundEnabled[4]={true,true,true,true};
uint8_t soundSensorIndex[4]={2,2,2,3}; // T3 for K1-K3; T4 for K4
float soundOnC[4]={NAN,NAN,NAN,NAN},soundOffC[4]={NAN,NAN,NAN,NAN};
bool soundGpioInitialized=false;
uint32_t soundLastPoll=0,soundSignalCount[4]={0},soundLastStartMs[4]={0};
uint32_t soundT3OnEvents=0,soundT3OffEvents=0,soundT4OnEvents=0,soundT4OffEvents=0;
int soundPinLevel(bool on){return BUSA_SOUND_ACTIVE_LOW?(on?LOW:HIGH):(on?HIGH:LOW);}
const char* soundRoleName(uint8_t i){
  const char* names[]={"K1_T3_CUBE_NORMAL_SOUND","K2_T3_OR_T4_ON_90S_SOUND",
    "K3_T3_OR_T4_OFF_90S_SOUND","K4_T4_TOP_WARNING_SOUND"};
  return i<4?names[i]:"UNKNOWN";
}
bool soundIsTimed(uint8_t i){return i==1||i==2;}
void soundOutputsOff(){
  for(int i=0;i<4;i++){
    if(soundGpioInitialized)digitalWrite(KPIN[i],soundPinLevel(false));
    soundChannels[i].on=false;
  }
  if(soundGpioInitialized)digitalWrite(BUSA_BUZZER_GPIO,buzzerPinLevel(false));
  buzzerSoundOn=false;
}
void soundInitPins(){
  // All four GPIOs connect exclusively to low-voltage 12 V sound relays.
  for(int i=0;i<4;i++){digitalWrite(KPIN[i],soundPinLevel(false));pinMode(KPIN[i],OUTPUT);}
  digitalWrite(BUSA_BUZZER_GPIO,buzzerPinLevel(false));
  pinMode(BUSA_BUZZER_GPIO,OUTPUT);
  soundGpioInitialized=true;soundOutputsOff();
}
void soundResetState(){
  BusaSound::resetRouted(soundRoute);
  for(int i=0;i<4;i++)soundChannels[i]=BusaSound::Channel();
  for(int i=0;i<2;i++)buzzerTempChannels[i]=BusaSound::Channel();
  buzzerSoundOn=false;
  soundLastPoll=0;
}
void soundLoadPrefs(){
  // Use NEW explicit T3/T4 namespace. Never turn old V14 thresholds into
  // immediate sound commands after a firmware update.
  soundOnC[0]=prefs.getFloat("sndT3On",NAN);
  soundOffC[0]=prefs.getFloat("sndT3Off",NAN);
  soundOnC[3]=prefs.getFloat("sndT4On",NAN);
  soundOffC[3]=prefs.getFloat("sndT4Off",NAN);
  for(int i=1;i<=2;i++){
    soundOnC[i]=soundOnC[0];soundOffC[i]=soundOffC[0];
  }
  for(int i=0;i<4;i++){
    soundEnabled[i]=prefs.getBool((String("sndRoleEnable")+i).c_str(),true);
    soundSensorIndex[i]=i==3?3:2;
  }
  for(int i=0;i<2;i++){
    buzzerEnabled[i]=prefs.getBool((String("buzE")+i).c_str(),true);
    buzzerOnC[i]=prefs.getFloat((String("buzOn")+i).c_str(),NAN);
    buzzerOffC[i]=prefs.getFloat((String("buzOff")+i).c_str(),NAN);
  }
  soundResetState();
}
void soundPoll(uint32_t now){
  if(!soundGpioInitialized)return;
  if(soundLastPoll && (uint32_t)(now-soundLastPoll)<BUSA_SOUND_POLL_MS)return;
  soundLastPoll=now;
  if(otaActive||otaRestart||tftRebootPending){
    soundOutputsOff();BusaSound::resetRouted(soundRoute);return;
  }
  const bool fresh3=hasTemp(2,now),fresh4=hasTemp(3,now);
  BusaSound::evaluateRouted(soundRoute,soundEnabled,
      fresh3,fresh3?temperature[2]:NAN,soundOnC[0],soundOffC[0],
      fresh4,fresh4?temperature[3]:NAN,soundOnC[3],soundOffC[3],
      now,BUSA_SOUND_TIMED_MS);
  if(soundRoute.t3OnEvent)++soundT3OnEvents;
  if(soundRoute.t3OffEvent)++soundT3OffEvents;
  if(soundRoute.t4OnEvent)++soundT4OnEvents;
  if(soundRoute.t4OffEvent)++soundT4OffEvents;
  for(int i=0;i<4;i++){
    const bool old=soundChannels[i].on;
    const bool on=soundRoute.k[i];
    soundChannels[i].on=on;
    const int sensor=soundSensorIndex[i];
    const bool known=BusaSound::valid(soundOnC[sensor==3?3:0],soundOffC[sensor==3?3:0]);
    soundChannels[i].reason=!soundEnabled[i]?BusaSound::Disabled:
      !known?BusaSound::MissingThreshold:!hasTemp(sensor,now)?BusaSound::SensorInvalid:
      on?BusaSound::Sounding:BusaSound::Waiting;
    if(on&&!old){soundLastStartMs[i]=now;++soundSignalCount[i];}
    digitalWrite(KPIN[i],soundPinLevel(on));
  }
  const bool previousBuzzer=buzzerSoundOn;
  bool sum=false;
  for(int i=0;i<2;i++){
    const bool fresh=hasTemp(i,now);
    const bool active=BusaSound::evaluate(buzzerTempChannels[i],buzzerEnabled[i],
       fresh,fresh?temperature[i]:NAN,buzzerOnC[i],buzzerOffC[i]);
    sum=sum||active;
  }
  buzzerSoundOn=sum; // T1 OR T2: independent thresholds, one sound output.
  if(sum&&!previousBuzzer){buzzerLastStartMs=now;++buzzerActivationCount;}
  digitalWrite(BUSA_BUZZER_GPIO,buzzerPinLevel(sum));
}
String soundJSON(uint32_t now){
  String s="{\"purpose\":\"12V_SOUND_ONLY\",\"active_low\":";
  s+=(BUSA_SOUND_ACTIVE_LOW?String("true"):String("false"));
  s+=",\"control_of_valve_or_power\":false";
  s+=",\"mapping\":\"T3_ON=K1+K2;T3_OFF=K3+K1_OFF;T4_ON=K4+K2;T4_OFF=K4_OFF+K3\"";
  s+=",\"t3_on_events\":"+String(soundT3OnEvents);
  s+=",\"t3_off_events\":"+String(soundT3OffEvents);
  s+=",\"t4_on_events\":"+String(soundT4OnEvents);
  s+=",\"t4_off_events\":"+String(soundT4OffEvents);
  s+=",\"k1_silenced_on_open\":"+(soundRoute.k1Hold?String("true"):String("false"));
  s+=",\"buzzer\":{\"gpio\":"+String(BUSA_BUZZER_GPIO);
  s+=",\"logic_voltage_v\":3.3,\"active_high\":true,\"source\":\"T1_OR_T2\"";
  s+=",\"sound_on\":"+(buzzerSoundOn?String("true"):String("false"));
  s+=",\"command_gpio_level\":"+String(buzzerPinLevel(buzzerSoundOn));
  s+=",\"activation_count\":"+String(buzzerActivationCount);
  s+=",\"last_start_ms\":"+String(buzzerLastStartMs);
  s+=",\"physical_buzzer_feedback\":false,\"sensors\":[";
  for(int i=0;i<2;i++){
    if(i)s+=",";
    s+="{\"sensor_index\":"+String(i);
    s+=",\"enabled\":"+(buzzerEnabled[i]?String("true"):String("false"));
    s+=",\"on_c\":"+(std::isfinite(buzzerOnC[i])?String(buzzerOnC[i],1):String("null"));
    s+=",\"off_c\":"+(std::isfinite(buzzerOffC[i])?String(buzzerOffC[i],1):String("null"));
    s+=",\"condition_active\":"+(buzzerTempChannels[i].on?String("true"):String("false"));
    s+=",\"reason\":\""+String(BusaSound::reasonText(buzzerTempChannels[i].reason))+"\"}";
  }
  s+="]}";
  s+=",\"channels\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    const uint8_t sensor=soundSensorIndex[i];
    const bool timed=soundIsTimed(i);
    const uint32_t remaining=i==1?BusaSound::remainingMs(soundRoute.k2,now,BUSA_SOUND_TIMED_MS):
      i==2?BusaSound::remainingMs(soundRoute.k3,now,BUSA_SOUND_TIMED_MS):0;
    s+="{\"channel\":"+String(i+1)+",\"gpio\":"+String(KPIN[i]);
    s+=",\"role\":\""+String(soundRoleName(i))+"\"";
    s+=",\"sensor_index\":"+String(sensor);
    s+=",\"temperature_gpio\":"+String(TPIN[sensor]);
    s+=",\"enabled\":"+(soundEnabled[i]?String("true"):String("false"));
    s+=",\"on_c\":"+(std::isfinite(soundOnC[sensor==3?3:0])?String(soundOnC[sensor==3?3:0],1):String("null"));
    s+=",\"off_c\":"+(std::isfinite(soundOffC[sensor==3?3:0])?String(soundOffC[sensor==3?3:0],1):String("null"));
    s+=",\"sound_on\":"+(soundChannels[i].on?String("true"):String("false"));
    s+=",\"sample_valid\":"+(hasTemp(sensor,now)?String("true"):String("false"));
    s+=",\"timed\":"+(timed?String("true"):String("false"));
    s+=",\"duration_seconds\":"+String(timed?90:0);
    s+=",\"remaining_seconds\":"+String((remaining+999UL)/1000UL);
    s+=",\"t3_condition\":"+(soundRoute.t3.on?String("true"):String("false"));
    s+=",\"t4_condition\":"+(soundRoute.t4.on?String("true"):String("false"));
    s+=",\"signal_count\":"+String(soundSignalCount[i]);
    s+=",\"last_signal_start_ms\":"+String(soundLastStartMs[i]);
    s+=",\"command_gpio_level\":"+String(soundPinLevel(soundChannels[i].on));
    s+=",\"physical_relay_feedback\":false";
    s+=",\"reason\":\""+String(BusaSound::reasonText(soundChannels[i].reason))+"\"}";
  }
  return s+"]}";
}
