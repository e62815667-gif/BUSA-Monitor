#pragma once
// All radio mutations belong to loop(), never the Wi-Fi event task.
// Keep AP+STA running: do not stop the radio or erase NVS to recover STA.
enum class BusaScanPhase : uint8_t { Idle, Settling, Running };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiPhaseAt=0,wifiConnectAt=0,wifiScanAt=0;
bool wifiConnectPending=false;
uint8_t wifiScanTries=0;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
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
  if(wifiScanPhase==BusaScanPhase::Running)esp_wifi_scan_stop();
  WiFi.scanDelete();wifiScanPhase=BusaScanPhase::Idle;
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  WiFi.scanDelete();wifiScanPhase=BusaScanPhase::Idle;
  if(WiFi.status()!=WL_CONNECTED){wifiConnectPending=true;wifiConnectAt=now;}
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy())return; // Idempotent: polling/repeated taps cannot reset the scan.
  wifiConnectPending=false;
  // A connection attempt takes precedence over scan_start in ESP-IDF.
  // End the attempt before scanning; an established connection can stay up.
  if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
  WiFi.scanDelete();wifiScanTries=0;
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
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  if(wifiScanBusy()){
    if(elapsed(now,wifiScanAt,15000)){
      esp_wifi_scan_stop();++wifiScanFailures;
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");return;
    }
    if(wifiScanPhase==BusaScanPhase::Settling){
      if(!elapsed(now,wifiPhaseAt,500))return;
      ++wifiScanTries;
      const int result=WiFi.scanNetworks(true,false,false,120);
      if(result==WIFI_SCAN_FAILED){
        ++wifiScanFailures;
        if(wifiScanTries>=3){
          wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_start_failed\",\"networks\":[]}");return;
        }
        if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
        wifiPhaseAt=now;return;
      }
      wifiScanPhase=BusaScanPhase::Running;
    }
    const int n=WiFi.scanComplete();
    if(n==WIFI_SCAN_RUNNING)return;
    if(n<0){
      ++wifiScanFailures;
      wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_failed\",\"networks\":[]}");return;
    }
    nearbyCount=0;
    String out="{\"scanning\":false,\"networks\":[";
    for(int i=0;i<n&&i<28;++i){
      const String ssid=WiFi.SSID(i);
      if(ssid.length()&&nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
      if(i)out+=",";
      out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String(WiFi.RSSI(i))+"}";
    }
    out+="]}";wifiFinishScan(now,out);return;
  }
  if(WiFi.status()==WL_CONNECTED){wifiConnectPending=false;lastStationCheck=now;return;}
  if(stationSsid.isEmpty()||stationPass.length()<8)return;
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  }else if(elapsed(now,lastStationCheck,20000)){
    wifiScheduleConnect(now); // Bounded attempts; leave an idle gap for user scans.
  }
}
