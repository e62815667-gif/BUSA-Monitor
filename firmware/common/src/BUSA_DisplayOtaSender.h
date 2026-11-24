#pragma once
#include "BUSA_DisplayOtaProtocol.h"
namespace busa_ota {
enum class Phase:uint8_t { idle, receiving, ready, sending, waitingBoot, success, error };
class Sender {
public:
 bool start(uint8_t* buffer,uint32_t size,uint32_t id,uint32_t now){
  if(busy()||!buffer||!validSize(size)||!id)return false;
  data_=buffer;size_=size;id_=id;received_=sent_=confirmed_=lastOffset_=0;at_=now;phase_=Phase::receiving;return true;
 }
 bool append(const uint8_t* data,size_t size,uint32_t now){
  if(phase_!=Phase::receiving||!data||!size||size>size_-received_){fail();return false;}
  memcpy(data_+received_,data,size);received_+=size;at_=now;return true;
 }
 bool readyToSeal()const{return phase_==Phase::receiving&&received_==size_;}
 bool seal(const uint8_t* hash,uint32_t now){
  if(!readyToSeal()||!hash||!validHeader(data_,size_)){fail();return false;}
  Marker display(displayFamily),main(mainFamily);display.feed(data_,size_);main.feed(data_,size_);
  if(!display.found()||main.found()){fail();return false;}
  memcpy(hash_,hash,sizeof(hash_));phase_=Phase::ready;at_=now;return true;
 }
 bool available()const{return phase_==Phase::ready||phase_==Phase::sending;}
 const uint8_t* block(uint32_t id,uint32_t offset,size_t& count)const{
  count=0;if(!available()||id!=id_||(offset!=sent_&&offset!=lastOffset_))return nullptr;
  count=chunkLength(size_,offset);return count?data_+offset:nullptr;
 }
 void sent(uint32_t offset,size_t count,uint32_t now){
  if(!available()||!count||count!=chunkLength(size_,offset)||(offset!=sent_&&offset!=lastOffset_))return;
  if(offset>confirmed_)confirmed_=offset;
  lastOffset_=offset;if(offset+count>sent_)sent_=offset+count;
  phase_=Phase::sending;at_=now;
 }
 bool accept(uint32_t id,uint8_t code,uint32_t now){
  if(id!=id_||!id||!busy()||phase_==Phase::receiving)return false;
  if(code==failed||code==bootFailed){fail();return true;}
  if(code==written&&(phase_==Phase::sending||phase_==Phase::waitingBoot)&&sent_==size_){confirmed_=size_;phase_=Phase::waitingBoot;at_=now;return true;}
  if(code==bootOk&&(phase_==Phase::waitingBoot||phase_==Phase::sending)&&sent_==size_){confirmed_=size_;phase_=Phase::success;return true;}
  return false;
 }
 void tick(uint32_t now){if(busy()&&uint32_t(now-at_)>=(phase_==Phase::waitingBoot?90000u:30000u))fail();}
 void fail(){if(busy())phase_=Phase::error;}
 bool busy()const{return phase_==Phase::receiving||available()||phase_==Phase::waitingBoot;}
 Phase phase()const{return phase_;}
 uint32_t id()const{return id_;} uint32_t size()const{return size_;}
 uint32_t received()const{return received_;} uint32_t confirmed()const{return confirmed_;}
 const uint8_t* data()const{return data_;} const uint8_t* hash()const{return hash_;}
private:
 Phase phase_=Phase::idle;uint8_t* data_=nullptr;uint8_t hash_[32]={};
 uint32_t size_=0,id_=0,received_=0,sent_=0,confirmed_=0,lastOffset_=0,at_=0;
};
}
