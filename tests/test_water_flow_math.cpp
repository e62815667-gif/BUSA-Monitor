#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "../firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_WaterFlowMath.h"
int main(){
  using namespace BusaWater;
  static_assert(kPassportPulsesPerLitre==5880.0f,"YF-S401 passport default");
  assert(std::fabs(lpm(196,2000)-1.0f)<0.0001f);
  assert(std::fabs(lpm(196,2000,5880.0f)-1.0f)<0.0001f);
  assert(std::fabs(lpm(196,2000,2940.0f)-2.0f)<0.0001f);
  assert(std::fabs(litres(5880,2940.0f)-2.0f)<0.0001f);
  assert(credible(196,2000,0,5880.0f));
  assert(!credible(196,2000,1,5880.0f));
  assert(!validPulsesPerLitre(0.0f));
  assert(!validPulsesPerLitre(100001.0f));
  assert(validPulsesPerLitre(5880.0f));
  assert(std::fabs(litres(5880)-1.0f)<0.0001f);
  assert(std::fabs(lpm(1176,2000)-6.0f)<0.0001f);
  assert(credible(196,2000,0));
  assert(credible(0,2000,0));
  assert(!credible(196,2000,1));
  assert(!credible(1400,2000,0));
  assert(!credible(1,0,0));
  assert(acceptEdge(100000,110205));
  assert(acceptEdge(100000,101700));
  assert(!acceptEdge(100000,100030));
  assert(!acceptEdge(100000,101100));
  assert(acceptEdge(UINT32_MAX-200,1100));
  assert(std::strcmp(diagnosticStatus(false,false,0),"WAITING_FOR_PULSES")==0);
  assert(std::strcmp(diagnosticStatus(true,false,250),"UNRELIABLE_PULSES")==0);
  assert(std::strcmp(diagnosticStatus(true,true,250),"PULSES_SEEN")==0);
  // A clean sample must recover current status after a noisy sample.
  assert(std::strcmp(diagnosticStatus(true,true,0),"NO_RECENT_PULSES")==0);
  return 0;
}
