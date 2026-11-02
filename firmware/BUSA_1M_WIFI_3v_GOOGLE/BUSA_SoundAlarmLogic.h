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
// Pure state machine for temperature-driven 12V SOUND signals only.
// T3: K1 continuous and K2 on T_ON; K3 on T_OFF.
// T4: K4 continuous and K2 on T_ON; K3 on T_OFF.
// Any K3 opening-sound event silences K1 until a NEW T3-on transition.
// No output can operate a valve or disconnect power.
struct Routed {
  Channel t3,t4;
  TimedSignal k2,k3;
  bool k1Hold=false;
  bool k[4]={false,false,false,false};
  bool t3OnEvent=false,t3OffEvent=false,t4OnEvent=false,t4OffEvent=false;
};
inline void resetRouted(Routed& r){r=Routed();}
inline void startPulse(TimedSignal& p,uint32_t now){
  p.running=true;p.sounding=true;p.start_ms=now;p.armed=false;
}
inline bool tickPulse(TimedSignal& p,uint32_t now,uint32_t durationMs){
  if(p.running && (uint32_t)(now-p.start_ms)>=durationMs)p.running=false;
  p.sounding=p.running;
  return p.sounding;
}
inline void evaluateRouted(Routed& r,const bool enable[4],
    bool t3Fresh,float t3,float t3On,float t3Off,
    bool t4Fresh,float t4,float t4On,float t4Off,
    uint32_t now,uint32_t pulseDurationMs=90000UL){
  const bool valid3=t3Fresh&&std::isfinite(t3)&&valid(t3On,t3Off);
  const bool valid4=t4Fresh&&std::isfinite(t4)&&valid(t4On,t4Off);
  const bool prev3=r.t3.on,prev4=r.t4.on;
  const bool active3=evaluate(r.t3,true,t3Fresh,t3,t3On,t3Off);
  const bool active4=evaluate(r.t4,true,t4Fresh,t4,t4On,t4Off);
  r.t3OnEvent=valid3&&!prev3&&active3;
  r.t3OffEvent=valid3&&prev3&&!active3;
  r.t4OnEvent=valid4&&!prev4&&active4;
  r.t4OffEvent=valid4&&prev4&&!active4;
  // Prevent an invalid sensor from generating false "open" signals.
  // Temperature telemetry remains independent and unchanged.
  if(!valid3||!valid4){
    // Do not turn ongoing pulses into new events. Missing data on one
    // temperature source does not disable the *other* source.
  }
  const bool openingEvent=r.t3OffEvent||r.t4OffEvent;
  const bool closingEvent=r.t3OnEvent||r.t4OnEvent;
  if(r.t3OnEvent)r.k1Hold=false;
  if(openingEvent){
    r.k1Hold=true;
    r.k2.running=false;r.k2.sounding=false;
    if(enable[2])startPulse(r.k3,now);
  }else if(closingEvent && enable[1] && !r.k3.running){
    startPulse(r.k2,now);
  }
  r.k[0]=enable[0]&&valid3&&active3&&!r.k1Hold;
  r.k[1]=enable[1]&&tickPulse(r.k2,now,pulseDurationMs);
  r.k[2]=enable[2]&&tickPulse(r.k3,now,pulseDurationMs);
  r.k[3]=enable[3]&&valid4&&active4;
  if(!enable[1])r.k2.running=r.k2.sounding=false;
  if(!enable[2])r.k3.running=r.k3.sounding=false;
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
