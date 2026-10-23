#pragma once
#include <new>
#include <cstring>
// V14.24: isolate SSID discovery from STA auto-reconnect (reason 201 on iPhone).
// The local access point remains active throughout. NVS credentials are retained.
// All radio mutations take place in the main Arduino loop, not in Wi-Fi events.
enum class BusaScanPhase : uint8_t { Idle, Settling, ArduinoRunning, DriverSettling };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiScanAt=0,wifiPhaseAt=0,wifiConnectAt=0;
bool wifiConnectPending=false,wifiScanUsedDriver=false;
uint8_t wifiScanTries=0;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0,wifiScanSuccesses=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
int wifiLastScanErr=0,wifiLastRecordsErr=0;
uint16_t wifiLastApCount=0,wifiLastRecordCount=0,wifiLastVisibleCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
constexpr uint32_t BUSA_SCAN_SETTLE_MS=1200U;
constexpr uint32_t BUSA_SCAN_TOTAL_TIMEOUT_MS=35000U;
constexpr uint8_t BUSA_SCAN_ARDUINO_TRIES=3;
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    ++wifiDisconnectCount;
  }
}
void wifiScheduleConnect(uint32_t now){
  wifiConnectPending=true;wifiConnectAt=now;
}
void wifiRestoreStationAfterScan(uint32_t now){
  if(!stationSsid.isEmpty() && stationPass.length()>=8){
    // Do not resume STA while scan results are still owned by the driver.
    WiFi.setAutoReconnect(true);
    if(WiFi.status()!=WL_CONNECTED){
      wifiConnectPending=true;wifiConnectAt=now;
    }else wifiConnectPending=false;
  }else{wifiConnectPending=false;WiFi.setAutoReconnect(false);}
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  WiFi.scanDelete();wifiScanPhase=BusaScanPhase::Idle;
  lastStationCheck=now;
  wifiRestoreStationAfterScan(now);
}
void wifiCancelScan(){
  if(wifiScanPhase==BusaScanPhase::ArduinoRunning)WiFi.scanDelete();
  // No dedicated esp_wifi_scan_stop here: a blocking IDF scan runs in loop().
  wifiScanPhase=BusaScanPhase::Idle;
  wifiRestoreStationAfterScan(millis());
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy() || otaActive || otaRestart)return;
  wifiScanCache="{\"scanning\":true,\"networks\":[]}";
  wifiScanTries=0;wifiScanUsedDriver=false;
  wifiLastScanErr=wifiLastRecordsErr=0;
  wifiLastApCount=wifiLastRecordCount=wifiLastVisibleCount=0;
  wifiConnectPending=false;
  WiFi.setAutoReconnect(false);
  // A pending Wi-Fi reconnect can make scanNetworks() return WIFI_SCAN_FAILED(-2).
  // Release the STA connection attempt while leaving WIFI_AP_STA and the local AP up.
  if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
  WiFi.scanDelete();
  wifiScanAt=wifiPhaseAt=now;wifiScanPhase=BusaScanPhase::Settling;
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
    ",\"scan_attempt\":"+String((unsigned)wifiScanTries)+
    ",\"ap_count\":"+String((unsigned)wifiLastApCount)+
    ",\"record_count\":"+String((unsigned)wifiLastRecordCount)+
    ",\"visible_ssid_count\":"+String((unsigned)wifiLastVisibleCount)+
    ",\"driver_fallback\":"+(wifiScanUsedDriver?"true":"false")+
    ",\"scan_engine\":\"arduino_with_idf_fallback\""+
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiStartDriverFallback(uint32_t now,int reason){
  wifiLastScanErr=reason;wifiScanUsedDriver=true;++wifiScanFailures;
  WiFi.scanDelete();
  wifiScanPhase=BusaScanPhase::DriverSettling;wifiPhaseAt=now;
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  if(wifiScanBusy()){
    if(elapsed(now,wifiScanAt,BUSA_SCAN_TOTAL_TIMEOUT_MS)){
      ++wifiScanFailures;
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");
      return;
    }
    if(wifiScanPhase==BusaScanPhase::Settling){
      if(!elapsed(now,wifiPhaseAt,BUSA_SCAN_SETTLE_MS))return;
      WiFi.enableSTA(true);
      ++wifiScanTries;
      const int rc=WiFi.scanNetworks(true,false,false,160);
      if(rc==WIFI_SCAN_FAILED){
        wifiLastScanErr=rc;++wifiScanFailures;
        if(wifiScanTries<BUSA_SCAN_ARDUINO_TRIES){
          // Leave the driver's busy state before the next attempt.
          if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
          WiFi.scanDelete();
          wifiPhaseAt=now;return;
        }
        wifiStartDriverFallback(now,rc);return;
      }
      wifiScanPhase=BusaScanPhase::ArduinoRunning;
      return;
    }
    if(wifiScanPhase==BusaScanPhase::ArduinoRunning){
      const int n=WiFi.scanComplete();
      if(n==WIFI_SCAN_RUNNING)return;
      if(n<0){wifiStartDriverFallback(now,n);return;}
      wifiLastApCount=(uint16_t)n;wifiLastRecordCount=(uint16_t)n;
      wifiLastVisibleCount=0;nearbyCount=0;
      String out="{\"scanning\":false,\"networks\":[";
      for(int i=0;i<n&&i<28;++i){
        const String ssid=WiFi.SSID(i);if(ssid.isEmpty())continue;
        ++wifiLastVisibleCount;
        if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
        if(out[out.length()-1]!='[')out+=",";
        out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String(WiFi.RSSI(i))+"}";
      }
      out+="]}";
      if(!wifiLastVisibleCount){wifiStartDriverFallback(now,0);return;}
      ++wifiScanSuccesses;wifiFinishScan(now,out);return;
    }
    if(wifiScanPhase==BusaScanPhase::DriverSettling){
      if(!elapsed(now,wifiPhaseAt,BUSA_SCAN_SETTLE_MS))return;
      WiFi.enableSTA(true);
      wifi_scan_config_t cfg={};
      cfg.show_hidden=false;cfg.scan_type=WIFI_SCAN_TYPE_ACTIVE;
      cfg.scan_time.active.min=100;cfg.scan_time.active.max=160;
      // Blocking IDF scan keeps result ownership in this call, avoiding async callback races.
      const esp_err_t rc=esp_wifi_scan_start(&cfg,true);
      now=millis();wifiLastScanErr=(int)rc;
      if(rc!=ESP_OK){
        ++wifiScanFailures;
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_driver_start_failed\",\"networks\":[]}");
        return;
      }
      uint16_t discovered=0;
      esp_err_t err=esp_wifi_scan_get_ap_num(&discovered);
      wifiLastRecordsErr=(int)err;
      if(err!=ESP_OK){
        ++wifiScanFailures;esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_driver_count_failed\",\"networks\":[]}");
        return;
      }
      wifiLastApCount=discovered;
      const uint16_t limit=discovered>28?28:discovered;
      wifi_ap_record_t* records=limit?new(std::nothrow) wifi_ap_record_t[limit]:nullptr;
      if(limit && !records){
        ++wifiScanFailures;esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_no_memory\",\"networks\":[]}");
        return;
      }
      uint16_t used=limit;
      err=limit?esp_wifi_scan_get_ap_records(&used,records):esp_wifi_clear_ap_list();
      wifiLastRecordsErr=(int)err;
      if(err!=ESP_OK){
        delete[] records;++wifiScanFailures;esp_wifi_clear_ap_list();
        wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_driver_results_failed\",\"networks\":[]}");
        return;
      }
      wifiLastRecordCount=used;wifiLastVisibleCount=0;nearbyCount=0;
      String out="{\"scanning\":false,\"networks\":[";
      for(uint16_t i=0;i<used;++i){
        char name[33];memcpy(name,records[i].ssid,32);name[32]=0;
        String ssid(name);if(ssid.isEmpty())continue;
        ++wifiLastVisibleCount;
        if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
        if(out[out.length()-1]!='[')out+=",";
        out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String((int)records[i].rssi)+"}";
      }
      delete[] records;
      out+="]}";++wifiScanSuccesses;wifiFinishScan(now,out);return;
    }
    return;
  }
  if(WiFi.status()==WL_CONNECTED){
    wifiConnectPending=false;lastStationCheck=now;return;
  }
  if(stationSsid.isEmpty()||stationPass.length()<8){wifiConnectPending=false;return;}
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,1500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.setAutoReconnect(true);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  }else if(elapsed(now,lastStationCheck,30000)){
    lastStationCheck=now;++wifiConnectAttempts;
    WiFi.setAutoReconnect(true);
    WiFi.reconnect();
  }
}
