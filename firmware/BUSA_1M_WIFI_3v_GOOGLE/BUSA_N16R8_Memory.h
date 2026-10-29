#pragma once
// BUSA-1M V15.2 / N16R8: bounded observation history in external PSRAM.
// Optional: if external PSRAM is not detected, the rest of firmware continues.
// Does not change relay GPIO, trigger logic, Google transport or phone app.
#include <esp_heap_caps.h>
struct BusaN16Sample {
  uint32_t uptime_ms;
  float temperature_c[4];
  float flow_lpm;
  uint32_t water_pulses;
};
constexpr size_t BUSA_PSRAM_SAMPLE_CAPACITY=512;
BusaN16Sample* busaN16History=nullptr;
size_t busaN16Samples=0;
size_t busaN16Head=0;
uint32_t busaN16LastSample=0;
bool busaN16MemoryReady=false;

void busaN16MemoryBegin(){
  busaN16MemoryReady=false;
  busaN16History=nullptr;
  busaN16Samples=0;
  busaN16Head=0;
  if(ESP.getPsramSize()<sizeof(BusaN16Sample)*BUSA_PSRAM_SAMPLE_CAPACITY) {
    Serial0.println("PSRAM: not detected or too small, history disabled");
    return;
  }
  busaN16History=static_cast<BusaN16Sample*>(heap_caps_calloc(
    BUSA_PSRAM_SAMPLE_CAPACITY,sizeof(BusaN16Sample),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
  busaN16MemoryReady=(busaN16History!=nullptr);
  Serial0.printf("PSRAM: bytes=%lu free=%lu sample_history=%s\n",
       (unsigned long)ESP.getPsramSize(),(unsigned long)ESP.getFreePsram(),
       busaN16MemoryReady?"OK":"ALLOC_FAILED");
}
void busaN16MemoryLog(uint32_t now,const float* temperatures,float flow_lpm,uint32_t pulses){
  if(!busaN16MemoryReady || !temperatures || (uint32_t)(now-busaN16LastSample)<15000U)return;
  busaN16LastSample=now;
  BusaN16Sample& point=busaN16History[busaN16Head];
  point.uptime_ms=now;
  for(int i=0;i<4;i++)point.temperature_c[i]=temperatures[i];
  point.flow_lpm=flow_lpm;
  point.water_pulses=pulses;
  busaN16Head=(busaN16Head+1)%BUSA_PSRAM_SAMPLE_CAPACITY;
  if(busaN16Samples<BUSA_PSRAM_SAMPLE_CAPACITY)++busaN16Samples;
}
