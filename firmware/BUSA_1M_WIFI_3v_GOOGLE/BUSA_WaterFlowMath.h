#pragma once
#include <stdint.h>

namespace BusaWater {
// YF-S401 (98 Hz per L/min, up to 6 L/min). The actual sensor still needs volume calibration.
constexpr float kHzPerLpm=98.0f;
constexpr float kPassportPulsesPerLitre=kHzPerLpm*60.0f;
// BUSA-1M #001: verified with a 950 mL / 60 s measured container on 2026-11-24.
constexpr float kBusa001VerifiedPulsesPerLitre=4710.0f;
inline bool verifiedBusa001Calibration(float ppl){
  return ppl>=kBusa001VerifiedPulsesPerLitre-0.05f &&
         ppl<=kBusa001VerifiedPulsesPerLitre+0.05f;
}
// Validate manual settings, including NaN and infinities, before division.
inline bool validPulsesPerLitre(float v){return v>=1.0f && v<=100000.0f;}
constexpr float kMaxPlausibleLpm=6.6f; // 10% tolerance above the 6 L/min specified range
constexpr uint32_t kMinEdgeUs=1200U; // shorter than the ~1701 us period at 6 L/min
inline bool acceptEdge(uint32_t previousUs,uint32_t nowUs){
  return previousUs==0U || (uint32_t)(nowUs-previousUs)>=kMinEdgeUs;
}
inline float lpm(uint32_t pulses,uint32_t dtMs,float pulsesPerLitre=kPassportPulsesPerLitre){
  return dtMs && validPulsesPerLitre(pulsesPerLitre)?
    ((float)pulses*60000.0f)/((float)dtMs*pulsesPerLitre):0.0f;
}
inline float litres(uint32_t pulses,float pulsesPerLitre=kPassportPulsesPerLitre){
  return validPulsesPerLitre(pulsesPerLitre)?(float)pulses/pulsesPerLitre:0.0f;
}
inline bool credible(uint32_t pulses,uint32_t dtMs,uint32_t rejectedEdges,float pulsesPerLitre=kPassportPulsesPerLitre){
  return dtMs && validPulsesPerLitre(pulsesPerLitre) && rejectedEdges==0U &&
    lpm(pulses,dtMs,pulsesPerLitre)<=kMaxPlausibleLpm;
}
// Only clean intervals contribute to the counted volume.
struct VolumeAccumulator {
  double countedLitres=0.0;
  uint32_t excludedIntervals=0;
  void record(uint32_t pulses,uint32_t rejectedEdges,bool credibleSample,float pulsesPerLitre){
    if(credibleSample)countedLitres+=litres(pulses,pulsesPerLitre);
    else if(pulses||rejectedEdges)++excludedIntervals;
  }
  bool complete()const{return excludedIntervals==0;}
};
// Current observation, independent of the session-wide volume validity.
inline const char* diagnosticStatus(bool sampleReady,bool credibleSample,uint32_t windowPulses){
  if(!sampleReady)return "WAITING_FOR_PULSES";
  if(!credibleSample)return "UNRELIABLE_PULSES";
  return windowPulses?"PULSES_SEEN":"NO_RECENT_PULSES";
}
}
