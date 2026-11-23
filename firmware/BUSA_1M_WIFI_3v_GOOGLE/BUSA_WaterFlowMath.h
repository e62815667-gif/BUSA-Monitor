#pragma once
#include <stdint.h>

namespace BusaWater {
// YF-S401 (98 Hz per L/min, up to 6 L/min). The actual sensor still needs volume calibration.
constexpr float kHzPerLpm=98.0f;
constexpr float kMaxPlausibleLpm=6.6f; // 10% tolerance above the 6 L/min specified range
constexpr uint32_t kMinEdgeUs=1200U; // shorter than the ~1701 us period at 6 L/min
inline bool acceptEdge(uint32_t previousUs,uint32_t nowUs){
  return previousUs==0U || (uint32_t)(nowUs-previousUs)>=kMinEdgeUs;
}
inline float lpm(uint32_t pulses,uint32_t dtMs){
  return dtMs?((float)pulses*1000.0f)/((float)dtMs*kHzPerLpm):0.0f;
}
inline float litres(uint32_t pulses){return (float)pulses/(kHzPerLpm*60.0f);}
inline bool credible(uint32_t pulses,uint32_t dtMs,uint32_t rejectedEdges){
  return dtMs && rejectedEdges==0U && lpm(pulses,dtMs)<=kMaxPlausibleLpm;
}
}
