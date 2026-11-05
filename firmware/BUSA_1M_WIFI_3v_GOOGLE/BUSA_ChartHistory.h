#pragma once
#include <stdint.h>
#include <math.h>
// Passive minute buckets. Caller owns memory; no GPIO/network/NVS access.
namespace BusaChart {
struct Row {uint32_t at;float t[4];};
class History {
 Row* rows=nullptr;uint16_t cap=0,head=0,count=0;
public:
 void begin(Row* storage,uint16_t capacity){rows=storage;cap=capacity;head=count=0;}
 void capture(uint32_t now,const float* values){
  if(!rows||!cap)return;
  uint16_t pos=(head+cap-1)%cap;
  if(!count||(uint32_t)(now-rows[pos].at)>=60000){
   pos=head;head=(head+1)%cap;if(count<cap)++count;rows[pos].at=now;
  }
  for(int k=0;k<4;k++)rows[pos].t[k]=isfinite(values[k])?values[k]:NAN;
 }
 uint16_t size()const{return count;}
 void select(uint32_t now,uint8_t hours,int16_t* indices,uint16_t bins)const{
  for(uint16_t b=0;b<bins;b++)indices[b]=-1;
  const uint32_t span=(uint32_t)hours*3600000u;
  if(!span||!bins)return;
  for(uint16_t n=0;n<count;n++){
   const uint16_t i=(head+cap-count+n)%cap;const uint32_t age=now-rows[i].at;
   if(age>=span)continue;
   uint16_t slot=(uint64_t)(span-1-age)*bins/span;
   if(slot>=bins)slot=bins-1;
   indices[slot]=(int16_t)i;
  }
 }
 float value(int16_t i,uint8_t k)const{return rows&&i>=0&&(uint16_t)i<cap&&k<4?rows[i].t[k]:NAN;}
};
}
