#pragma once
// BUSA 1M: four independent 12V SOUND-ONLY alarm channels.
// Thresholds are separate from legacy V14 display reference keys.
#include <cmath>
#include <cstdint>
namespace BusaSound {
enum Reason : uint8_t { Disabled, MissingThreshold, SensorInvalid, Waiting, Sounding };
struct Channel { bool on=false; Reason reason=Disabled; };
inline bool valid(float on,float off) {
  return std::isfinite(on) && std::isfinite(off) && on>=0 && on<=125 &&
    off>=0 && off<=125 && std::fabs(on-off)>=0.5f;
}
inline bool evaluate(Channel& c,bool enabled,bool fresh,float t,float on,float off){
  if(!enabled){c.on=false;c.reason=Disabled;return false;}
  if(!valid(on,off)){c.on=false;c.reason=MissingThreshold;return false;}
  if(!fresh||!std::isfinite(t)){c.on=false;c.reason=SensorInvalid;return false;}
  if(on>off){
    if(!c.on && t>=on)c.on=true;
    else if(c.on && t<=off)c.on=false;
  }else{
    if(!c.on && t<=on)c.on=true;
    else if(c.on && t>=off)c.on=false;
  }
  c.reason=c.on?Sounding:Waiting;
  return c.on;
}
// Sound-only pulse logic for K2 and K3. 90 s once on each new
// temperature-condition transition; rearm only after T_OFF is reached.
struct TimedSignal {
  bool armed=true;
  bool running=false;
  bool sounding=false;
  uint32_t start_ms=0;
};
inline void resetTimed(TimedSignal& p){ p=TimedSignal(); }
inline bool evaluateTimed(Channel& threshold,TimedSignal& pulse,
    bool enabled,bool fresh,float t,float on,float off,
    uint32_t now,uint32_t durationMs){
  const bool condition=evaluate(threshold,enabled,fresh,t,on,off);
  if(!condition){
    pulse.armed=true;
    pulse.running=false;
    pulse.sounding=false;
    return false;
  }
  if(pulse.armed){
    pulse.armed=false;
    pulse.running=true;
    pulse.start_ms=now;
  }
  pulse.sounding=pulse.running && (uint32_t)(now-pulse.start_ms)<durationMs;
  if(!pulse.sounding)pulse.running=false;
  return pulse.sounding;
}
inline uint32_t remainingMs(const TimedSignal& pulse,uint32_t now,uint32_t durationMs){
  if(!pulse.running || !pulse.sounding)return 0;
  const uint32_t elapsed=(uint32_t)(now-pulse.start_ms);
  return elapsed>=durationMs?0:(durationMs-elapsed);
}
inline const char* reasonText(Reason r){
  switch(r){
    case Disabled:return "DISABLED";
    case MissingThreshold:return "SETPOINTS_REQUIRED";
    case SensorInvalid:return "SENSOR_INVALID";
    case Waiting:return "WAITING";
    case Sounding:return "SOUNDING";
  }
  return "UNKNOWN";
}
}
