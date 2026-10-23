#pragma once
// V14.18: scan networks through ESP-IDF, not Arduino WiFiScan's mirrored
// asynchronous flags. Event callback records only basic completion state;
// all Wi-Fi driver calls and JSON allocation stay on the Arduino loop task.
// No passwords are exposed or erased, AP+STA stays enabled.
enum class BusaScanPhase : uint8_t { Idle, Settling, Running };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiPhaseAt=0,wifiConnectAt=0,wifiScanAt=0,lastScanStartedAt=0;
bool wifiConnectPending=false;
uint8_t wifiScanTries=0;
constexpr uint8_t WIFI_SCAN_MAX_TRIES=4;
constexpr uint32_t WIFI_SCAN_SETTLE_MS=750,WIFI_SCAN_TIMEOUT_MS=19000;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0,wifiScanSuccesses=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
volatile bool wifiScanDoneEvent=false;
volatile uint32_t wifiScanDoneStatus=0;
int wifiLastScanErr=0,wifiLastRecordsErr=0;
uint16_t wifiLastApCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    ++wifiDisconnectCount;
  }else if(event==ARDUINO_EVENT_WIFI_SCAN_DONE){
    wifiScanDoneStatus=info.wifi_scan_done.status;
    wifiScanDoneEvent=true;
  }
}
void wifiScheduleConnect(uint32_t now){
  WiFi.disconnect(false,false);
  wifiConnectPending=true;wifiConnectAt=now;
}
void wifiCancelScan(){
  if(wifiScanPhase==BusaScanPhase::Running)esp_wifi_scan_stop();
  esp_wifi_clear_ap_list();
  wifiScanPhase=BusaScanPhase::Idle;
  wifiScanDoneEvent=false;
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  wifiScanPhase=BusaScanPhase::Idle;
  wifiScanDoneEvent=false;
  if(WiFi.status()!=WL_CONNECTED && !stationSsid.isEmpty() && stationPass.length()>=8){
    wifiConnectPending=true;wifiConnectAt=now;
  }
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy())return;
  wifiConnectPending=false;
  // Pre-empt a pending STA connection before starting the scan.
  if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
  wifiScanDoneEvent=false;wifiScanDoneStatus=0;
  wifiScanTries=0;wifiLastScanErr=0;wifiLastRecordsErr=0;wifiLastApCount=0;
  wifiScanPhase=BusaScanPhase::Settling;wifiPhaseAt=now;wifiScanAt=now;
  wifiScanCache="{\"scanning\":true,\"networks\":[]}";
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
    ",\"scan_done_status\":"+String((uint32_t)wifiScanDoneStatus)+
    ",\"ap_count\":"+String((unsigned)wifiLastApCount)+
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiScanRetryOrFail(uint32_t now,const char* error){
  ++wifiScanFailures;
  esp_wifi_clear_ap_list();
  wifiScanDoneEvent=false;
  if(wifiScanTries<WIFI_SCAN_MAX_TRIES && !elapsed(now,wifiScanAt,WIFI_SCAN_TIMEOUT_MS-1000)){
    wifiScanPhase=BusaScanPhase::Settling;
    wifiPhaseAt=now;
    return;
  }
  wifiFinishScan(now,String("{\"scanning\":false,\"error\":")+quoteJson(error)+",\"networks\":[]}");
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  if(wifiScanBusy()){
    if(elapsed(now,wifiScanAt,WIFI_SCAN_TIMEOUT_MS)){
      if(wifiScanPhase==BusaScanPhase::Running)esp_wifi_scan_stop();
      ++wifiScanFailures;
      esp_wifi_clear_ap_list();
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");
      return;
    }
    if(wifiScanPhase==BusaScanPhase::Settling){
      if(!elapsed(now,wifiPhaseAt,WIFI_SCAN_SETTLE_MS))return;
      ++wifiScanTries;
      wifiScanDoneEvent=false;wifiScanDoneStatus=0;
      // Zero-init is required; old Arduino wrapper could return FAILED before
      // a completed scan was observed on an unconfigured STA.
      WiFi.enableSTA(true);
      wifi_scan_config_t config={};
      config.show_hidden=false;
      config.scan_type=WIFI_SCAN_TYPE_ACTIVE;
      config.scan_time.active.min=100;
      config.scan_time.active.max=180;
      const esp_err_t rc=esp_wifi_scan_start(&config,false);
      wifiLastScanErr=(int)rc;
      if(rc!=ESP_OK){wifiScanRetryOrFail(now,"scan_driver_start_failed");return;}
      wifiScanPhase=BusaScanPhase::Running;
      lastScanStartedAt=now;
      return;
    }
    if(!wifiScanDoneEvent)return;
    // The Wi-Fi event task only sets flags; consume results on the loop task.
    const uint32_t done=wifiScanDoneStatus;
    wifiScanDoneEvent=false;
    if(done!=0){
      wifiLastRecordsErr=(int)done;
      wifiScanRetryOrFail(now,"scan_driver_event_failed");
      return;
    }
    uint16_t discovered=0;
    esp_err_t rc=esp_wifi_scan_get_ap_num(&discovered);
    wifiLastRecordsErr=(int)rc;
    if(rc!=ESP_OK){wifiScanRetryOrFail(now,"scan_records_count_failed");return;}
    wifiLastApCount=discovered;
    const uint16_t limit=discovered>20?20:discovered;
    wifi_ap_record_t* records=limit?new(std::nothrow) wifi_ap_record_t[limit]:nullptr;
    if(limit && !records){
      esp_wifi_clear_ap_list();
      wifiScanRetryOrFail(now,"scan_memory_unavailable");return;
    }
    uint16_t used=limit;
    if(limit)rc=esp_wifi_scan_get_ap_records(&used,records);
    else rc=esp_wifi_clear_ap_list();
    wifiLastRecordsErr=(int)rc;
    if(rc!=ESP_OK){
      delete[] records;wifiScanRetryOrFail(now,"scan_records_read_failed");return;
    }
    nearbyCount=0;
    String out="{\"scanning\":false,\"networks\":[";
    for(uint16_t i=0;i<used;i++){
      const String ssid=String((const char*)records[i].ssid);
      if(!ssid.length())continue;
      if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
      if(out[out.length()-1]!='[')out+=",";
      out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String((int)records[i].rssi)+"}";
    }
    delete[] records;
    out+="]}";
    ++wifiScanSuccesses;
    wifiFinishScan(now,out);
    return;
  }
  if(WiFi.status()==WL_CONNECTED){wifiConnectPending=false;lastStationCheck=now;return;}
  if(stationSsid.isEmpty()||stationPass.length()<8){
    wifiConnectPending=false;
    return;
  }
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  }else if(elapsed(now,lastStationCheck,20000)){
    wifiScheduleConnect(now);
  }
}
