#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <sdkconfig.h>
#include <mbedtls/sha256.h>
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define BUSA_SHA_START mbedtls_sha256_starts
#define BUSA_SHA_UPDATE mbedtls_sha256_update
#define BUSA_SHA_FINISH mbedtls_sha256_finish
#else
#define BUSA_SHA_START mbedtls_sha256_starts_ret
#define BUSA_SHA_UPDATE mbedtls_sha256_update_ret
#define BUSA_SHA_FINISH mbedtls_sha256_finish_ret
#endif
#include <BUSA_DisplayOtaProtocol.h>
#include <BUSA_DisplayProtocol.h>
#if !CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
#error Display OTA requires the ESP32-S3 rollback-enabled SDK
#endif
namespace busa_guition {
class DisplayOtaReceiver {
 using Frame=busa_uart::Frame;
 struct BootRecord{uint32_t id,address;};
public:
 void begin(){
  bootAt_=millis();prefsReady_=prefs_.begin("busa-dota",false);
  if(prefsReady_&&prefs_.getBytesLength("pending")==sizeof(record_))
   if(prefs_.getBytes("pending",&record_,sizeof(record_))!=sizeof(record_))record_={};
  const auto* running=esp_ota_get_running_partition();esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
  const bool haveState=running&&esp_ota_get_state_partition(running,&state)==ESP_OK;
  pendingBoot_=haveState&&(state==ESP_OTA_IMG_PENDING_VERIFY||(!record_.id&&state==ESP_OTA_IMG_UNDEFINED));
  if(record_.id&&(!running||running->address!=record_.address)){id_=record_.id;report_=busa_ota::bootFailed;pendingBoot_=false;}
  else if(record_.id&&!pendingBoot_){
   id_=record_.id;report_=state==ESP_OTA_IMG_VALID?busa_ota::bootOk:busa_ota::bootFailed;
  }
 }
 bool priority()const{return active_||report_;}
 bool receiving()const{return active_;}
 bool blocksApi()const{return active_||report_==busa_ota::written;}
 void tick(uint32_t now,bool localReady,bool connected){
  if(active_&&uint32_t(now-progressAt_)>=30000)fail();
  if(!pendingBoot_)return;
  const bool initialInstall=!record_.id&&!esp_ota_check_rollback_is_possible();
  if(localReady&&(connected||initialInstall)&&esp_ota_mark_app_valid_cancel_rollback()==ESP_OK){
   pendingBoot_=false;
   if(record_.id){id_=record_.id;report_=busa_ota::bootOk;}
  }else if(uint32_t(now-bootAt_)>=30000&&esp_ota_check_rollback_is_possible()){
   esp_ota_mark_app_invalid_rollback_and_reboot();
  }
 }
 bool next(uint32_t now,bool supported,bool connected,bool localReady,uint8_t& kind,uint8_t& expected,uint8_t* data,size_t& size){
  using busa_uart::write32;
  size=0;
  if(report_){
   if((report_==busa_ota::bootOk||report_==busa_ota::bootFailed)&&(!supported||!connected))return false;
   kind=busa_ota::result;expected=busa_ota::ack;write32(data,id_);data[4]=report_;size=5;return true;
  }
  if(active_){kind=busa_ota::request;expected=busa_ota::chunk;write32(data,id_);write32(data+4,received_);size=8;return true;}
  if(!supported||!connected||!localReady||!prefsReady_||pendingBoot_||record_.id||uint32_t(now-lastPoll_)<1000)return false;
  lastPoll_=now;kind=busa_ota::poll;expected=busa_ota::offer;return true;
 }
 bool reply(const Frame& f){
  if(f.kind==busa_ota::offer){if(f.size==busa_ota::offerSize)start(f);return true;}
  if(f.kind==busa_ota::chunk){receive(f);return true;}
  if(f.kind==busa_ota::ack){
   if(f.size!=4||busa_uart::read32(f.data)!=id_||!report_)return true;
   if(report_==busa_ota::written){ESP.restart();return true;}
   if(report_==busa_ota::bootOk||report_==busa_ota::bootFailed){
    if(prefsReady_){prefs_.remove("pending");}
    record_={};
   }
   report_=0;return true;
  }
  return false;
 }
 void timeout(uint8_t expected){
  if(expected==busa_ota::chunk&&active_)fail();
  if(expected==busa_ota::ack&&report_==busa_ota::written)ESP.restart();
 }
private:
 void start(const Frame& f){
  using busa_uart::read32;
  const uint32_t id=read32(f.data),size=read32(f.data+4);
  if(!id||active_||report_||pendingBoot_||record_.id)return;
  id_=id;size_=size;received_=0;marker_.reset();memcpy(expected_,f.data+8,32);
  const auto* running=esp_ota_get_running_partition();target_=esp_ota_get_next_update_partition(nullptr);
  if(!prefsReady_||!busa_ota::validSize(size)||!running||!target_||target_->address==running->address||size>target_->size){
   report_=busa_ota::failed;return;
  }
  if(esp_ota_begin(target_,size,&handle_)!=ESP_OK){report_=busa_ota::failed;return;}
  active_=true;mbedtls_sha256_init(&sha_);
  if(BUSA_SHA_START(&sha_,0)!=0){fail();return;}
  progressAt_=millis();
 }
 void receive(const Frame& f){
  using busa_uart::read32;
  if(!active_)return;
  const size_t count=busa_ota::chunkLength(size_,received_);
  if(!count||f.size!=8+count||read32(f.data)!=id_||read32(f.data+4)!=received_){fail();return;}
  const auto* data=f.data+8;
  if((received_==0&&!busa_ota::validHeader(data,count))||
     BUSA_SHA_UPDATE(&sha_,data,count)!=0||esp_ota_write(handle_,data,count)!=ESP_OK){fail();return;}
  marker_.feed(data,count);received_+=count;progressAt_=millis();
  if(received_==size_)finish();
 }
 void finish(){
  uint8_t actual[32];
  if(BUSA_SHA_FINISH(&sha_,actual)!=0||memcmp(actual,expected_,32)!=0||!marker_.found()){fail();return;}
  const esp_err_t rc=esp_ota_end(handle_);active_=false;mbedtls_sha256_free(&sha_);
  if(rc!=ESP_OK){report_=busa_ota::failed;return;}
  const BootRecord record{id_,target_->address};BootRecord check{};
  if(prefs_.putBytes("pending",&record,sizeof(record))!=sizeof(record)||
     prefs_.getBytes("pending",&check,sizeof(check))!=sizeof(check)||check.id!=record.id||check.address!=record.address||
     esp_ota_set_boot_partition(target_)!=ESP_OK){prefs_.remove("pending");report_=busa_ota::failed;return;}
  record_=record;report_=busa_ota::written;
 }
 void fail(){
  if(active_){esp_ota_abort(handle_);mbedtls_sha256_free(&sha_);active_=false;}
  report_=busa_ota::failed;
 }
 Preferences prefs_;BootRecord record_{};
 bool prefsReady_=false,active_=false,pendingBoot_=false;
 uint8_t report_=0,expected_[32]={};
 uint32_t id_=0,size_=0,received_=0,bootAt_=0,progressAt_=0,lastPoll_=0;
 const esp_partition_t* target_=nullptr;esp_ota_handle_t handle_=0;
 mbedtls_sha256_context sha_;
 busa_ota::Marker marker_{busa_ota::displayFamily};
};
}

#undef BUSA_SHA_START
#undef BUSA_SHA_UPDATE
#undef BUSA_SHA_FINISH
