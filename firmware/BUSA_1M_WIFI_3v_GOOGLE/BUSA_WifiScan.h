#pragma once
#include <atomic>
#include <esp_wifi.h>
// One async scan owner for web and TFT. Arduino's SCAN_DONE collector runs BEFORE
// user WiFi.onEvent callbacks in core 2.0.17; keep using its SSID/RSSI storage.
// Bypass only scanNetworks' uninitialised wifi_scan_config_t, not the SDK/core.
enum BusaSharedScanState : uint8_t {BUSA_SCAN_IDLE=0,BUSA_SCAN_REQUESTED=1,BUSA_SCAN_RUNNING=2,BUSA_SCAN_READY=3,BUSA_SCAN_EMPTY=4,BUSA_SCAN_ERROR=5,BUSA_SCAN_STOPPING=6};
BusaSharedScanState busaSharedScanState=BUSA_SCAN_IDLE;
uint8_t busaSharedScanAttempt=0;uint32_t busaSharedScanAttemptAt=0,busaSharedScanFinishedAt=0,busaSharedScanRetryDueAt=0,busaSharedScanStopAt=0;
bool busaSharedScanForTft=false,busaSharedScanReleasedSTA=false;
int busaSharedScanStartResult=0,busaSharedScanLastResult=0;
String busaSharedScanError="";
static std::atomic<bool> busaScanAcceptEvent{false},busaScanEventReady{false};
static std::atomic<uint32_t> busaScanEventStatus{0},busaScanEventNumber{0};
static bool busaScanObserverReady=false;
static void busaWifiScanEvent(arduino_event_t* event){
 if(event&&event->event_id==ARDUINO_EVENT_WIFI_SCAN_DONE&&busaScanAcceptEvent.load()){
  busaScanEventStatus=event->event_info.wifi_scan_done.status;
  busaScanEventNumber=event->event_info.wifi_scan_done.number;
  busaScanEventReady=true;
 }
}
static void busaWifiInitScanner(){
 if(!busaScanObserverReady)busaScanObserverReady=WiFi.onEvent(busaWifiScanEvent,ARDUINO_EVENT_WIFI_SCAN_DONE)!=0;
}
static uint32_t busaWifiDeadline(uint32_t now,uint32_t delayMs){uint32_t due=now+delayMs;return due?due:1;}
bool busaWifiRequestSharedScan(bool physical,bool fresh){
 uint32_t now=millis();if(otaActive)return false;
 if(physical)busaSharedScanForTft=true;
 if(busaWifiScanRunning)return true;
 if(!fresh&&busaSharedScanState==BUSA_SCAN_READY&&uint32_t(now-busaSharedScanFinishedAt)<5000){
  if(physical){wifiUiStage=WIFIUI_NETWORKS;wifiUiMenuIndex=0;wifiUiIndex=0;wifiUiFeedback="";busaSharedScanForTft=false;tftDirty=true;}
  return true;
 }
 // A connected STA is NEVER disconnected for discovery, including empty retries.
 // Cancel only an unfinished association, then resume the CURRENT saved target.
 bool pending=wifiUiConnectPending||wifiUiAssociationDueAt||wifiUiConnectAt;
 wifiUiConnectPending=false;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;
 WiFi.setAutoReconnect(false);busaSharedScanReleasedSTA=WiFi.status()!=WL_CONNECTED;
 if(busaSharedScanReleasedSTA)WiFi.disconnect(false,false);
 busaSharedScanRetryDueAt=busaWifiDeadline(now,(busaSharedScanReleasedSTA||pending)?900:1);
 busaSharedScanAttempt=0;busaSharedScanStartResult=0;busaSharedScanLastResult=0;busaSharedScanError="";
 busaSharedScanState=BUSA_SCAN_REQUESTED;busaWifiScanRunning=true;busaWifiScanStartedAt=now;wifiUiCount=0;
 return true;
}
void busaWifiScanFinish(uint8_t state,uint32_t now){
 busaScanAcceptEvent=false;
 // Normal completion arrives after Arduino's record collector. On lost-event
 // timeout leave buffers alone until the next request, after the stop grace.
 if(busaScanEventReady.load()||state!=BUSA_SCAN_ERROR)WiFi.scanDelete();
 busaWifiScanRunning=false;busaSharedScanState=(BusaSharedScanState)state;
 busaSharedScanFinishedAt=now;busaWifiScanLastAt=now;wifiUiScanAt=now;
 wifiUiScanAttempts=busaSharedScanAttempt;
 wifiUiLastScanCode=state==BUSA_SCAN_READY?wifiUiCount:state==BUSA_SCAN_EMPTY?0:busaSharedScanError=="scan_timeout"?-3:-2;
 if(busaSharedScanForTft&&wifiUiStage==WIFIUI_SCANNING&&currentPage==5&&approvedSettingsItem==1&&tftView){
  wifiUiStage=wifiUiCount?WIFIUI_NETWORKS:WIFIUI_STATUS;wifiUiIndex=0;wifiUiMenuIndex=0;
  wifiUiFeedback=wifiUiCount?String(""):state==BUSA_SCAN_EMPTY?String("Сети не найдены"):String("Ошибка Wi-Fi: ")+busaSharedScanError;
  tftDirty=true;
 }
 busaSharedScanForTft=false;
 // wifiUiPoll's bounded supervisor reconnects only stationSsid, unless staOff.
}
void busaWifiScanPoll(uint32_t now){
 if(busaSharedScanState==BUSA_SCAN_STOPPING){
  if(busaScanEventReady.load()||uint32_t(now-busaSharedScanStopAt)>=1000)busaWifiScanFinish(BUSA_SCAN_ERROR,now);
  return;
 }
 if(busaSharedScanState==BUSA_SCAN_REQUESTED){
  if(otaActive){busaSharedScanError="scan_cancelled_ota";busaWifiScanFinish(BUSA_SCAN_ERROR,now);return;}
  if(wifiUiDisablePending)return; // apply explicit disconnect BEFORE starting RF scan
  if(busaSharedScanRetryDueAt&&int32_t(now-busaSharedScanRetryDueAt)<0)return;
  if(!busaScanObserverReady){busaSharedScanError="scan_event_unavailable";busaWifiScanFinish(BUSA_SCAN_ERROR,now);return;}
  if(WiFi.getMode()!=WIFI_AP_STA&&!WiFi.mode(WIFI_AP_STA)){busaSharedScanError="sta_unavailable";busaWifiScanFinish(BUSA_SCAN_ERROR,now);return;}
  WiFi.setSleep(false);WiFi.scanDelete();
  ++busaSharedScanAttempt;busaSharedScanAttemptAt=now;busaSharedScanRetryDueAt=0;
  busaScanEventReady=false;busaScanEventStatus=0;busaScanEventNumber=0;busaScanAcceptEvent=true;
  wifi_scan_config_t config={};config.channel=0;config.show_hidden=true;
  config.scan_type=busaSharedScanAttempt>1?WIFI_SCAN_TYPE_PASSIVE:WIFI_SCAN_TYPE_ACTIVE;
  config.scan_time.active.min=120;config.scan_time.active.max=300;config.scan_time.passive=450;
  config.home_chan_dwell_time=60;
  const esp_err_t rc=esp_wifi_scan_start(&config,false);busaSharedScanStartResult=rc;
  if(rc==ESP_OK){busaSharedScanState=BUSA_SCAN_RUNNING;return;}
  busaScanAcceptEvent=false;busaSharedScanError="scan_start_failed";
  if(busaSharedScanAttempt<2){busaSharedScanRetryDueAt=busaWifiDeadline(now,900);return;}
  busaWifiScanFinish(BUSA_SCAN_ERROR,now);return;
 }
 if(busaSharedScanState!=BUSA_SCAN_RUNNING)return;
 if(otaActive||uint32_t(now-busaSharedScanAttemptAt)>=10000){
  busaSharedScanError=otaActive?"scan_cancelled_ota":"scan_timeout";
  busaSharedScanStopAt=now;busaSharedScanState=BUSA_SCAN_STOPPING;esp_wifi_scan_stop();return;
 }
 if(!busaScanEventReady.load())return;
 const int count=WiFi.scanComplete();busaSharedScanLastResult=count;
 const uint32_t eventStatus=busaScanEventStatus.load(),eventNumber=busaScanEventNumber.load();
 if(eventStatus!=0||count<0||(count==0&&eventNumber>0)){
  busaSharedScanError=eventStatus?"scan_driver_failed":"scan_results_unavailable";
 }else if(count==0)busaSharedScanError="no_networks";
 else{
  wifiUiCount=0;
  for(int i=0;i<count&&wifiUiCount<48;i++){
   String name=WiFi.SSID(i);if(name.isEmpty()||name.length()>32)continue;
   bool duplicate=false;for(int j=0;j<wifiUiCount;j++)if(wifiUiNetworks[j]==name){duplicate=true;break;}
   if(duplicate)continue;wifiUiNetworks[wifiUiCount]=name;wifiUiSignals[wifiUiCount]=WiFi.RSSI(i);++wifiUiCount;
  }
  busaSharedScanError=wifiUiCount?"":"only_hidden_networks";
  busaWifiScanFinish(wifiUiCount?BUSA_SCAN_READY:BUSA_SCAN_EMPTY,now);return;
 }
 if(busaSharedScanAttempt<2){
  busaScanAcceptEvent=false;WiFi.scanDelete();busaSharedScanRetryDueAt=busaWifiDeadline(now,900);busaSharedScanState=BUSA_SCAN_REQUESTED;return;
 }
 busaWifiScanFinish(busaSharedScanError=="no_networks"?BUSA_SCAN_EMPTY:BUSA_SCAN_ERROR,now);
}
String busaWifiScanJSON(){
 String out=String("{\"scanning\":")+(busaWifiScanRunning?"true":"false")+",\"networks\":[";
 if(!busaWifiScanRunning&&busaSharedScanState==BUSA_SCAN_READY)for(int i=0;i<wifiUiCount;i++){
  if(i)out+=",";out+="{\"ssid\":"+quoteJson(wifiUiNetworks[i])+",\"rssi\":"+String(wifiUiSignals[i])+"}";
 }
 out+="],\"error\":"+quoteJson(busaWifiScanRunning?String(""):busaSharedScanError);
 out+=",\"diagnostic\":{\"start_api\":\"esp_wifi_scan_start\",\"attempt\":"+String(busaSharedScanAttempt);
 out+=",\"start_result\":"+String(busaSharedScanStartResult)+",\"scan_result\":"+String(busaSharedScanLastResult);
 out+=",\"event_status\":"+String(busaScanEventStatus.load())+",\"event_number\":"+String(busaScanEventNumber.load());
 out+=",\"wifi_mode\":"+String((int)WiFi.getMode())+",\"station_status\":"+String((int)WiFi.status())+"}}";return out;
}
