#pragma once
#include <stdint.h>
#include <string.h>
namespace BusaWebJournal {
struct Entry {uint32_t seq,uptime,epoch;uint8_t kind;char text[112];};
class Ring {
 Entry* rows=nullptr;uint16_t capacity=0,head=0,count=0;uint32_t sequence=0;
public:
 void begin(Entry* storage,uint16_t cap){rows=storage;capacity=cap;head=count=0;sequence=0;}
 void add(uint32_t uptime,uint32_t epoch,uint8_t kind,const char* text){
  if(!rows||!capacity||!text||!*text)return;
  auto& row=rows[head];row.seq=++sequence;row.uptime=uptime;row.epoch=epoch;row.kind=kind;
  size_t len=strlen(text);if(len>=sizeof(row.text))len=sizeof(row.text)-1;
  // Never leave a truncated UTF-8 continuation sequence at the end.
  while(len&&((uint8_t)text[len]&0xc0)==0x80)--len;
  memcpy(row.text,text,len);row.text[len]=0;
  head=(head+1)%capacity;if(count<capacity)++count;
 }
 uint16_t size()const{return count;}
 const Entry* latest(uint16_t n)const{return rows&&n<count?&rows[(head+capacity-1-n)%capacity]:nullptr;}
};
}
