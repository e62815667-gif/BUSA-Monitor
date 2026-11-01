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
