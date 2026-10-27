#pragma once
// Pure C++ four-channel temperature sound-alarm hysteresis.
// No hardware dependencies: compiled and tested with host g++.
#include <cmath>
#include <cstdint>
namespace BusaAlarm {
enum Reason : uint8_t { Disabled, HardwareLocked, MissingThreshold, SensorInvalid, Waiting, Sounding };
struct Channel {
  bool on=false;
  Reason reason=Disabled;
};
inline bool validPair(float turnOn,float turnOff){
  return std::isfinite(turnOn)&&std::isfinite(turnOff)&&
    turnOn>=0.0f&&turnOn<=125.0f&&turnOff>=0.0f&&turnOff<=125.0f&&
    std::fabs(turnOn-turnOff)>=0.5f;
}
inline bool evaluate(Channel& state,bool enabled,bool verified,float c,float turnOn,float turnOff){
  if(!enabled){state.on=false;state.reason=Disabled;return false;}
  if(!validPair(turnOn,turnOff)){state.on=false;state.reason=MissingThreshold;return false;}
  if(!verified||!std::isfinite(c)){state.on=false;state.reason=SensorInvalid;return false;}
  if(turnOn>turnOff){
    if(!state.on && c>=turnOn)state.on=true;         // hot alarm
    else if(state.on && c<=turnOff)state.on=false;
  }else{
    if(!state.on && c<=turnOn)state.on=true;         // cold alarm
    else if(state.on && c>=turnOff)state.on=false;
  }
  state.reason=state.on?Sounding:Waiting;
  return state.on;
}
inline const char* reasonText(Reason r){
  switch(r){
    case Disabled:return "DISABLED";
    case HardwareLocked:return "HARDWARE_LOCKED";
    case MissingThreshold:return "SETPOINTS_REQUIRED";
    case SensorInvalid:return "SENSOR_INVALID";
    case Waiting:return "WAITING";
    case Sounding:return "SOUNDING";
  }
  return "UNKNOWN";
}
}
