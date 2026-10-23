#pragma once
#include <new>
#include <cstring>
// V14.21: use V14.9 Arduino scan first, with a synchronous ESP-IDF fallback.
// All radio mutations stay in loop(); keep the protected AP and saved STA credentials.
enum class BusaScanPhase : uint8_t { Idle, Settling, ArduinoRunning, IdfSettling };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiPhaseAt=0,wifiConnectAt=0,wifiScanAt=0;
bool wifiConnectPending=false,wifiLastUsedFallback=false;
uint8_t wifiScanTries=0;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0,wifiScanSuccesses=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
int wifiLastScanErr=0,wifiLastRecordsErr=0;
uint16_t wifiLastApCount=0,wifiLastRecordCount=0,wifiLastVisibleCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
constexpr uint32_t WIFI_SCAN_SETTLE_MS=650, WIFI_SCAN_TIMEOUT_MS=30000;
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    ++wifiDisconnectCount;
  }
}
void wifiScheduleConnect(uint32_t now){
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false,false);
  wifiConnectPending=true;wifiConnectAt=now;
}
void wifiCancelScan(){
  if(wifiScanPhase==BusaScanPhase::ArduinoRunning)WiFi.scanDelete();
  wifiScanPhase=BusaScanPhase::Idle;
  WiFi.setAutoReconnect(!stationSsid.isEmpty() && stationPass.length()>=8);
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  WiFi.scanDelete();wifiScanPhase=BusaScanPhase::Idle;
  WiFi.setAutoReconnect(!stationSsid.isEmpty() && stationPass.length()>=8);
  if(WiFi.status()!=WL_CONNECTED && !stationSsid.isEmpty() && stationPass.length()>=8){
    wifiConnectPending=true;wifiConnectAt=now;
  }else wifiConnectPending=false;
}
void wifiStartIdfFallback(uint32_t now,int err){
  wifiLastUsedFallback=true;wifiLastScanErr=err;
  ++wifiScanFailures;
  WiFi.scanDelete();
  wifiScanPhase=BusaScanPhase::IdfSettling;wifiPhaseAt=now;
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy())return;
  wifiConnectPending=false;
  WiFi.setAutoReconnect(false);
  // An in-flight STA connection can prevent the driver from starting a scan.
  // Leave established STA connection and the local AP running.
  if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
  WiFi.scanDelete();
  wifiScanTries=0;wifiLastUsedFallback=false;
  wifiLastScanErr=0;wifiLastRecordsErr=0;
  wifiLastApCount=wifiLastRecordCount=wifiLastVisibleCount=0;
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
    ",\"scan_start_error\":"+String(wifiLastScanErr)+
    ",\"scan_result_error\":"+String(wifiLastRecordsErr)+
    ",\"ap_count\":"+String((unsigned)wifiLastApCount)+
    ",\"record_count\":"+String((unsigned)wifiLastRecordCount)+
    ",\"visible_ssid_count\":"+String((unsigned)wifiLastVisibleCount)+
    ",\"driver_fallback\":"+(wifiLastUsedFallback?"true":"false")+
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  if(wifiScanBusy()){
    if(elapsed(now,wifiScanAt,WIFI_SCAN_TIMEOUT_MS)){
      ++wifiScanFailures;
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");
      return;
    }
    if(wifiScanPhase==BusaScanPhase::Settling){
      if(!elapsed(now,wifiPhaseAt,WIFI_SCAN_SETTLE_MS))return;
      ++wifiScanTries;
      WiFi.enableSTA(true);
      // V14.9 discovered nearby networks using Arduino's scan cache.
      // Keep the same API but guard it against concurrent STA retries.
      const int rc=WiFi.scanNetworks(true,false,false,160);
      if(rc==WIFI_SCAN_FAILED){
        wifiStartIdfFallback(now,rc);return;
      }
      wifiScanPhase=BusaScanPhase::ArduinoRunning;
      return;
    }
    if(wifiScanPhase==BusaScanPhase::ArduinoRunning){
      const int n=WiFi.scanComplete();
      if(n==WIFI_SCAN_RUNNING)return;
      if(n<0){wifiStartIdfFallback(now,n);return;}
      wifiLastApCount=(uint16_t)n;wifiLastRecordCount=(uint16_t)n;
      wifiLastVisibleCount=0;nearbyCount=0;
      String out="{\"scanning\":false,\"networks\":[";
      for(int i=0;i<n&&i<28;++i){
        String ssid=WiFi.SSID(i);
        if(ssid.isEmpty())continue;
        ++wifiLastVisibleCount;
        if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
        // Count only emitted records: hidden/blank SSIDs must not
        // introduce an invalid leading comma in the JSON array.
        if(out[out.length()-1]!='[')out+=",";
        out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String(WiFi.RSSI(i))+"}";
      }
      out+="]}";
      // A successful scan with zero visible networks is not yet a proof
      // that no hotspot exists: try independent IDF enumeration once.
      if(!wifiLastVisibleCount){wifiStartIdfFallback(now,0);return;}
      ++wifiScanSuccesses;
      wifiFinishScan(now,out);return;
    }
    if(wifiScanPhase==BusaScanPhase::IdfSettling){
      if(!elapsed(now,wifiPhaseAt,WIFI_SCAN_SETTLE_MS))return;
      WiFi.enableSTA(true);
      wifi_scan_config_t config={};
      config.show_hidden=false;config.scan_type=WIFI_SCAN_TYPE_ACTIVE;
      config.scan_time.active.min=110;config.scan_time.active.max=180;
      // Blocking IDF scans do not race Arduino's async SCAN_DONE consumer.
      const esp_err_t rc=esp_wifi_scan_start(&config,true);
      now=millis();wifiLastScanErr=(int)rc;
      if(rc!=ESP_OK){
        ++wifiScanFailures;wifiFinishScan(now,"{\"scanning\":false,\"error\":\"driver_scan_start_failed\",\"networks\":[]}");
        return;
      }
      uint16_t found=0;esp_err_t rc2=esp_wifi_scan_get_ap_num(&found);
      wifiLastRecordsErr=(int)rc2;
      if(rc2!=ESP_OK){
        ++wifiScanFailures;esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"driver_scan_count_failed\",\"networks\":[]}");
        return;
      }
      wifiLastApCount=found;
      const uint16_t limit=found>28?28:found;
      wifi_ap_record_t* records=limit?new(std::nothrow) wifi_ap_record_t[limit]:nullptr;
      if(limit && !records){
        ++wifiScanFailures;esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_out_of_memory\",\"networks\":[]}");
        return;
      }
      uint16_t used=limit;
      rc2=limit?esp_wifi_scan_get_ap_records(&used,records):esp_wifi_clear_ap_list();
      wifiLastRecordsErr=(int)rc2;
      if(rc2!=ESP_OK){
        delete[] records;++wifiScanFailures;
        esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"driver_scan_records_failed\",\"networks\":[]}");
        return;
      }
      wifiLastRecordCount=used;wifiLastVisibleCount=0;nearbyCount=0;
      String out="{\"scanning\":false,\"networks\":[";
      for(uint16_t i=0;i<used;i++){
        char ssidBytes[33];
        memcpy(ssidBytes,records[i].ssid,32);ssidBytes[32]=0;
        String ssid(ssidBytes);
        if(ssid.isEmpty())continue;
        ++wifiLastVisibleCount;
        if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
        if(out[out.length()-1]!='[')out+=",";
        out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String((int)records[i].rssi)+"}";
      }
      delete[] records;
      out+="]}";
      ++wifiScanSuccesses;wifiFinishScan(now,out);return;
    }
    return;
  }
  if(WiFi.status()==WL_CONNECTED){wifiConnectPending=false;lastStationCheck=now;return;}
  if(stationSsid.isEmpty()||stationPass.length()<8){wifiConnectPending=false;return;}
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.setAutoReconnect(true);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  }else if(elapsed(now,lastStationCheck,20000)){
    wifiScheduleConnect(now);
  }
}
