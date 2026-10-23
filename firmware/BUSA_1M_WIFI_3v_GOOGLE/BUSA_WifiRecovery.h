#pragma once
#include <new>
// V14.18: use an ESP-IDF *blocking* scan in the loop task.
// Arduino ESP32 2.0.17 already consumes async WIFI_SCAN_DONE internally;
// fetching its AP records again would race that callback. With block=true
// IDF does not emit SCAN_DONE and the scan caller owns the results.
// Keep the existing protected AP up and preserve STA/Google credentials.
enum class BusaScanPhase : uint8_t { Idle, Settling };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiPhaseAt=0,wifiConnectAt=0,wifiScanAt=0;
bool wifiConnectPending=false;
uint8_t wifiScanTries=0;
constexpr uint8_t WIFI_SCAN_MAX_TRIES=3;
constexpr uint32_t WIFI_SCAN_SETTLE_MS=650,WIFI_SCAN_TIMEOUT_MS=25000;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0,wifiScanSuccesses=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
int wifiLastScanErr=0,wifiLastRecordsErr=0;
uint16_t wifiLastApCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    ++wifiDisconnectCount;
  }
}
void wifiScheduleConnect(uint32_t now){
  WiFi.disconnect(false,false);
  wifiConnectPending=true;wifiConnectAt=now;
}
void wifiCancelScan(){
  // No scan runs after wifiRecoveryTick returns; no concurrent cancellation.
  wifiScanPhase=BusaScanPhase::Idle;
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  wifiScanPhase=BusaScanPhase::Idle;
  if(WiFi.status()!=WL_CONNECTED && !stationSsid.isEmpty() && stationPass.length()>=8){
    wifiConnectPending=true;wifiConnectAt=now;
  }else wifiConnectPending=false;
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy())return;
  wifiConnectPending=false;
  if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
  wifiScanTries=0;wifiLastScanErr=0;wifiLastRecordsErr=0;wifiLastApCount=0;
  wifiScanPhase=BusaScanPhase::Settling;wifiPhaseAt=now;wifiScanAt=now;
}
String wifiScanJSON(){
  return wifiScanBusy()?String("{\"scanning\":true,\"networks\":[]}"):wifiScanCache;
}
String wifiRecoveryJSON(){
  return String("{\"scan_active\":")+(wifiScanBusy()?"true":"false")+
    ",\"connect_pending\":"+(wifiConnectPending?"true":"false")+
    ",\"attempts\":"+String(wifiConnectAttempts)+
    ",\"scan_failures\":"+String(wifiScanFailures)+
    ",\"scan_successes\":"+String(wifiScanSuccesses)+
    ",\"scan_attempt\":"+String((unsigned)wifiScanTries)+
    ",\"scan_start_error\":"+String(wifiLastScanErr)+
    ",\"scan_result_error\":"+String(wifiLastRecordsErr)+
    ",\"ap_count\":"+String((unsigned)wifiLastApCount)+
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiScanRetryOrFail(uint32_t now,const char* error){
  ++wifiScanFailures;
  esp_wifi_clear_ap_list();
  if(wifiScanTries<WIFI_SCAN_MAX_TRIES && !elapsed(now,wifiScanAt,WIFI_SCAN_TIMEOUT_MS-5000)){
    wifiScanPhase=BusaScanPhase::Settling;wifiPhaseAt=now;return;
  }
  wifiFinishScan(now,String("{\"scanning\":false,\"error\":")+quoteJson(error)+",\"networks\":[]}");
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  if(wifiScanBusy()){
    if(elapsed(now,wifiScanAt,WIFI_SCAN_TIMEOUT_MS)){
      ++wifiScanFailures;esp_wifi_clear_ap_list();
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");
      return;
    }
    if(!elapsed(now,wifiPhaseAt,WIFI_SCAN_SETTLE_MS))return;
    ++wifiScanTries;
    WiFi.enableSTA(true);
    wifi_scan_config_t config={};
    config.show_hidden=false;
    config.scan_type=WIFI_SCAN_TYPE_ACTIVE;
    config.scan_time.active.min=100;
    config.scan_time.active.max=160;
    // A blocking scan does not trigger WIFI_EVENT_SCAN_DONE, so it cannot
    // race the Arduino WiFiGeneric callback which normally drains scan results.
    const esp_err_t rc=esp_wifi_scan_start(&config,true);
    wifiLastScanErr=(int)rc;
    now=millis(); // scan may have taken several seconds
    if(rc!=ESP_OK){wifiScanRetryOrFail(now,"scan_driver_start_failed");return;}
    uint16_t discovered=0;
    esp_err_t err=esp_wifi_scan_get_ap_num(&discovered);
    wifiLastRecordsErr=(int)err;
    if(err!=ESP_OK){wifiScanRetryOrFail(now,"scan_records_count_failed");return;}
    wifiLastApCount=discovered;
    const uint16_t limit=discovered>20?20:discovered;
    wifi_ap_record_t* records=limit?new(std::nothrow) wifi_ap_record_t[limit]:nullptr;
    if(limit&&!records){wifiScanRetryOrFail(now,"scan_memory_unavailable");return;}
    uint16_t used=limit;
    err=limit?esp_wifi_scan_get_ap_records(&used,records):esp_wifi_clear_ap_list();
    wifiLastRecordsErr=(int)err;
    if(err!=ESP_OK){delete[] records;wifiScanRetryOrFail(now,"scan_records_read_failed");return;}
    nearbyCount=0;
    String out="{\"scanning\":false,\"networks\":[";
    for(uint16_t i=0;i<used;i++){
      String ssid=String((const char*)records[i].ssid);
      if(ssid.isEmpty())continue;
      if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
      if(out[out.length()-1]!='[')out+=",";
      out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String((int)records[i].rssi)+"}";
    }
    delete[] records;
    out+="]}";++wifiScanSuccesses;
    wifiFinishScan(now,out);return;
  }
  if(WiFi.status()==WL_CONNECTED){wifiConnectPending=false;lastStationCheck=now;return;}
  if(stationSsid.isEmpty()||stationPass.length()<8){wifiConnectPending=false;return;}
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  }else if(elapsed(now,lastStationCheck,20000)){
    wifiScheduleConnect(now);
  }
}
