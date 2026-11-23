#pragma once
#include <stdint.h>

namespace BusaWater {
// YF-S401 (98 Hz per L/min, up to 6 L/min). The actual sensor still needs volume calibration.
constexpr float kHzPerLpm=98.0f;
constexpr float kPassportPulsesPerLitre=kHzPerLpm*60.0f;
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
}
