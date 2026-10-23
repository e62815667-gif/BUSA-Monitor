#pragma once
// V14.22: restore the proven V14.14 Arduino Wi-Fi scanning/connection path.
// Scans are started by the local /api/wifi/scan handler using
// WiFi.scanDelete(); WiFi.scanNetworks(true,false).
// Polling reads WiFi.scanComplete() and returns the cached Arduino results.
// No direct ESP-IDF scan_start or competing scan state machines.
enum class BusaScanPhase : uint8_t { Idle, Running };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
uint32_t wifiScanAt=0,wifiConnectAt=0;
bool wifiConnectPending=false;
uint32_t wifiConnectAttempts=0,wifiScanFailures=0,wifiScanSuccesses=0;
volatile uint32_t wifiDisconnectCount=0;
volatile uint8_t wifiDisconnectReason=0;
int wifiLastScanErr=0,wifiLastRecordsErr=0;
uint16_t wifiLastApCount=0,wifiLastRecordCount=0,wifiLastVisibleCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason=info.wifi_sta_disconnected.reason;
    ++wifiDisconnectCount;
  }
}
void wifiFinishScan(uint32_t now,const String& result){
  wifiScanCache=result;
  WiFi.scanDelete();
  wifiScanPhase=BusaScanPhase::Idle;
  lastStationCheck=now;
  WiFi.setAutoReconnect(!stationSsid.isEmpty() && stationPass.length()>=8);
}
void wifiCancelScan(){
  WiFi.scanDelete();
  wifiScanPhase=BusaScanPhase::Idle;
  WiFi.setAutoReconnect(!stationSsid.isEmpty() && stationPass.length()>=8);
}
void wifiScheduleConnect(uint32_t now){
  // Retained for existing code integrations. Do not start a connect while scanning.
  wifiConnectPending=true;wifiConnectAt=now;
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy())return; // Repeat taps cannot delete an active scan.
  wifiConnectPending=false;
  wifiLastScanErr=wifiLastRecordsErr=0;
  wifiLastApCount=wifiLastRecordCount=wifiLastVisibleCount=0;
  wifiScanCache="{\"scanning\":true,\"networks\":[]}";
  WiFi.scanDelete();
  // Exactly the Arduino WiFiClass async scan API used in V14.14.
  const int rc=WiFi.scanNetworks(true,false);
  wifiScanAt=now;
  if(rc==WIFI_SCAN_FAILED){
    ++wifiScanFailures;wifiLastScanErr=rc;
    wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_start_failed\",\"networks\":[]}");
    return;
  }
  wifiScanPhase=BusaScanPhase::Running;
}
String wifiScanJSON(){
  if(!wifiScanBusy())return wifiScanCache;
  const uint32_t now=millis();
  if(elapsed(now,wifiScanAt,20000)){
    ++wifiScanFailures;
    wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_timeout\",\"networks\":[]}");
    return wifiScanCache;
  }
  const int n=WiFi.scanComplete();
  if(n==WIFI_SCAN_RUNNING)return "{\"scanning\":true,\"networks\":[]}";
  if(n<0){
    ++wifiScanFailures;wifiLastRecordsErr=n;
    wifiFinishScan(now,"{\"scanning\":false,\"error\":\"scan_failed\",\"networks\":[]}");
    return wifiScanCache;
  }
  wifiLastApCount=(uint16_t)n;wifiLastRecordCount=(uint16_t)n;
  wifiLastVisibleCount=0;nearbyCount=0;
  String out="{\"scanning\":false,\"networks\":[";
  for(int i=0;i<n&&i<28;i++){
    const String ssid=WiFi.SSID(i);
    if(ssid.isEmpty())continue;
    ++wifiLastVisibleCount;
    if(nearbyCount<5)nearbyNames[nearbyCount++]=ssid;
    if(out[out.length()-1]!='[')out+=",";
    out+="{\"ssid\":"+quoteJson(ssid)+",\"rssi\":"+String(WiFi.RSSI(i))+"}";
  }
  out+="]}";
  ++wifiScanSuccesses;
  wifiFinishScan(now,out);
  return wifiScanCache;
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
    ",\"scan_engine\":\"arduino_v1414\""+
    ",\"disconnects\":"+String((uint32_t)wifiDisconnectCount)+
    ",\"last_disconnect_reason\":"+String((unsigned)wifiDisconnectReason)+"}";
}
void wifiRecoveryTick(uint32_t now){
  if(otaActive||otaRestart)return;
  // No background reconnect/disconnect while the 14.14 scan is active.
  if(wifiScanBusy())return;
  if(WiFi.status()==WL_CONNECTED){
    wifiConnectPending=false;lastStationCheck=now;return;
  }
  if(stationSsid.isEmpty()||stationPass.length()<8){wifiConnectPending=false;return;}
  if(wifiConnectPending){
    if(!elapsed(now,wifiConnectAt,500))return;
    wifiConnectPending=false;lastStationCheck=now;++wifiConnectAttempts;
    WiFi.setAutoReconnect(true);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    return;
  }
  if(elapsed(now,lastStationCheck,20000)){
    lastStationCheck=now;++wifiConnectAttempts;
    WiFi.setAutoReconnect(true);
    WiFi.reconnect();
  }
}
