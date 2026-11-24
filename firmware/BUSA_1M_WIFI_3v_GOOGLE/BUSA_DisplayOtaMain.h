#pragma once
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>
#include <BUSA_DisplayOtaSender.h>
#include <BUSA_DisplayProtocol.h>
namespace busa_display_ota {
using namespace busa_ota;
static Sender sender;
static uint8_t* storage=nullptr;
static void release(){if(!sender.busy()&&storage){free(storage);storage=nullptr;}}
static bool busy(){return sender.busy();}
static bool begin(uint32_t size){
 if(busy()||!validSize(size))return false;
 release();storage=static_cast<uint8_t*>(heap_caps_malloc(size,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(!storage)return false;
 uint32_t id=esp_random();if(!id)id=1;
 if(!sender.start(storage,size,id,millis())){free(storage);storage=nullptr;return false;}return true;
}
static bool append(const uint8_t* data,size_t size){
 const bool ok=sender.append(data,size,millis());release();return ok;
}
static void abort(){sender.fail();release();}
static bool seal(){
 uint8_t hash[32];
 if(!sender.readyToSeal()||mbedtls_sha256_ret(sender.data(),sender.size(),hash,0)!=0){abort();return false;}
 const bool ok=sender.seal(hash,millis());release();return ok;
}
static void tick(){sender.tick(millis());release();}
template<class Enqueue> static bool handle(const busa_uart::Frame& f,Enqueue send){
 using busa_uart::read32;using busa_uart::write32;
 if(f.kind==busa_ota::poll){
  if(f.size)return true;
  uint8_t data[offerSize]={};
  if(sender.available()){
   write32(data,sender.id());write32(data+4,sender.size());memcpy(data+8,sender.hash(),32);
   send(offer,f.seq,data,sizeof(data));
  }else send(offer,f.seq,nullptr,0);
  return true;
 }
 if(f.kind==request){
  if(f.size!=requestSize)return true;
  const uint32_t id=read32(f.data),offset=read32(f.data+4);size_t count=0;
  const auto* source=sender.block(id,offset,count);
  if(!source){send(chunk,f.seq,nullptr,0);return true;}
  uint8_t data[requestSize+blockSize];
  write32(data,id);write32(data+4,offset);memcpy(data+8,source,count);
  if(send(chunk,f.seq,data,count+8))sender.sent(offset,count,millis());
  return true;
 }
 if(f.kind==result){
  if(f.size!=resultSize)return true;
  const uint8_t code=f.data[4];if(code<written||code>bootFailed)return true;
  sender.accept(read32(f.data),code,millis());
  send(ack,f.seq,f.data,4);release();return true;
 }
 return false;
}
static String status(bool capable,const char* version){
 static const char* names[]={"idle","receiving","ready","sending","waiting_boot","success","error"};
 String j="{\"capable\":"+String(capable?"true":"false")+",\"state\":\""+names[uint8_t(sender.phase())]+"\",\"id\":"+String(sender.id());
 j+=",\"size\":"+String(sender.size())+",\"received\":"+String(sender.received())+",\"confirmed\":"+String(sender.confirmed());
 // displayFirmware is sanitized by the existing diagnostic decoder.
 j+=",\"firmware\":\""+String(version?version:"")+"\"}";return j;
}
}
