#pragma once
#include <stdint.h>
#include <math.h>
// Passive timed samples; optional aggregation interval. Caller owns memory; no GPIO/network/NVS access.
namespace BusaChart {
struct Row {uint32_t at;float t[4];};
class History {
 Row* rows=nullptr;uint16_t cap=0,head=0,count=0;uint32_t bucketMs=60000;
public:
 void begin(Row* storage,uint16_t capacity,uint32_t bucket=60000){rows=storage;cap=capacity;bucketMs=bucket;head=count=0;}
 void capture(uint32_t now,const float* values){
  if(!rows||!cap)return;
  uint16_t pos=(head+cap-1)%cap;
  if(!count||!bucketMs||(uint32_t)(now-rows[pos].at)>=bucketMs){
   pos=head;head=(head+1)%cap;if(count<cap)++count;rows[pos].at=now;
  }
  for(int k=0;k<4;k++)rows[pos].t[k]=isfinite(values[k])?values[k]:NAN;
 }
 uint16_t size()const{return count;}
 void selectMs(uint32_t now,uint32_t span,int16_t* indices,uint16_t bins)const{
  for(uint16_t b=0;b<bins;b++)indices[b]=-1;
  if(!span||!bins)return;
  for(uint16_t n=0;n<count;n++){
   const uint16_t i=(head+cap-count+n)%cap;const uint32_t age=now-rows[i].at;
   if(age>=span)continue;
   uint16_t slot=(uint64_t)(span-1-age)*bins/span;
   if(slot>=bins)slot=bins-1;
   indices[slot]=(int16_t)i;
  }
 }
 void select(uint32_t now,uint8_t hours,int16_t* indices,uint16_t bins)const{selectMs(now,(uint32_t)hours*3600000u,indices,bins);}
 float value(int16_t i,uint8_t k)const{return rows&&i>=0&&(uint16_t)i<cap&&k<4?rows[i].t[k]:NAN;}
};
}
