#include <cassert>
#include <cmath>
#include <cstdint>
#include "../firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_WaterFlowMath.h"
int main(){
  using namespace BusaWater;
  assert(std::fabs(lpm(196,2000)-1.0f)<0.0001f);
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
  return 0;
}
