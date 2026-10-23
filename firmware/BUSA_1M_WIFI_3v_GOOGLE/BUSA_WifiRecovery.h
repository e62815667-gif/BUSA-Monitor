#pragma once
#include <atomic>
class BusaWiFiScanControl : public WiFiGenericClass {
public:
  static void clearScanBusy(){clearStatusBits(WIFI_SCANNING_BIT);}
};
// V14.27: AP+STA coexistence; local clients never block STA association.
// One loop owns radio work; no AP restart or forced home-channel reset.
// IDF migrates AP to STA channel using CSA. Manual scans visit one channel at a time.
enum class BusaScanPhase : uint8_t { Idle, Requested, Settling, Running, HomePause };
BusaScanPhase wifiScanPhase=BusaScanPhase::Idle;
constexpr uint8_t BUSA_AP_CHANNEL=6;
constexpr uint32_t BUSA_BOOT_GRACE_MS=3000U,BUSA_HTTP_REPLY_GRACE_MS=1500U;
constexpr uint32_t BUSA_CONNECT_TIMEOUT_MS=12000U,BUSA_SCAN_TIMEOUT_MS=12000U;
constexpr uint32_t BUSA_SCAN_SETTLE_MS=1200U,BUSA_SCAN_HOME_MS=200U;
constexpr uint8_t BUSA_SCAN_MAX_TRIES=2;
uint32_t wifiScanAt=0,wifiPhaseAt=0,wifiConnectAt=0,wifiPolicyAt=0;
uint32_t wifiRetryAt=0,wifiRetryDelay=0,wifiLastScanRequest=0;
bool wifiConnectPending=false,wifiConnecting=false,wifiJoinSettling=false,wifiHadScan=false;
bool wifiDeferredForLocal=false,wifiCancelRequested=false;
uint8_t wifiScanTries=0,wifiScanChannel=1,wifiScanLastChannel=11,wifiFailureStreak=0;
uint8_t wifiRadioChannel=0;
uint32_t wifiConnectAttempts=0,wifiConnectFailures=0,wifiScanFailures=0,wifiScanSuccesses=0;
uint32_t wifiAttemptDisconnects=0,wifiChannelChanges=0,wifiLastChannelCheck=0;
std::atomic<uint32_t> wifiDisconnectCount{0},wifiDisconnectReason{0};
std::atomic<uint32_t> wifiApStarts{0},wifiApStops{0},wifiApJoins{0},wifiApLeaves{0};
std::atomic<bool> wifiCloudPaused{false};
int wifiLastScanErr=0;
uint16_t wifiLastApCount=0,wifiLastVisibleCount=0;
struct BusaNetwork {String ssid;int32_t rssi=0;uint8_t channel=0;};
BusaNetwork wifiNetworks[28];
uint8_t wifiNetworkCount=0;
String wifiScanCache="{\"scanning\":false,\"networks\":[]}";
bool wifiScanBusy(){return wifiScanPhase!=BusaScanPhase::Idle;}
void wifiRecoveryEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  // Event task records facts only. No begin/reconnect/scan/reset here.
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    wifiDisconnectReason.store(info.wifi_sta_disconnected.reason);
    wifiDisconnectCount.fetch_add(1);
  }else if(event==ARDUINO_EVENT_WIFI_AP_START)wifiApStarts.fetch_add(1);
  else if(event==ARDUINO_EVENT_WIFI_AP_STOP)wifiApStops.fetch_add(1);
  else if(event==ARDUINO_EVENT_WIFI_AP_STACONNECTED)wifiApJoins.fetch_add(1);
  else if(event==ARDUINO_EVENT_WIFI_AP_STADISCONNECTED)wifiApLeaves.fetch_add(1);
}
bool wifiHasCredentials(){return !stationSsid.isEmpty() && stationPass.length()>=8;}
void wifiScheduleConnect(uint32_t now){
  // Called by HTTP/setup: schedule only; response is sent before radio work.
  wifiConnectPending=wifiHasCredentials();wifiRetryAt=now;wifiRetryDelay=BUSA_HTTP_REPLY_GRACE_MS;
  wifiConnecting=wifiJoinSettling=false;wifiFailureStreak=0;
}
void wifiStartPolicy(uint32_t now){
  wifiPolicyAt=now;WiFi.setAutoReconnect(false);
  wifiScheduleConnect(now);
}
void wifiStopScan(){
  // scanDelete alone does not stop the IDF radio scan or clear SCANNING_BIT.
  if(wifiScanPhase==BusaScanPhase::Running){
    esp_wifi_scan_stop();
    BusaWiFiScanControl::clearScanBusy();
  }
  WiFi.scanDelete();wifiScanPhase=BusaScanPhase::Idle;
}
void wifiCancelScan(){
  wifiStopScan();
  wifiScanCache="{\"scanning\":false,\"error\":\"scan_cancelled\",\"networks\":[]}";
  wifiCloudPaused.store(false);
}
void wifiFinishScan(const char* error=nullptr){
  String out="{\"scanning\":false";
  if(error){out+=",\"error\":"+quoteJson(String(error));++wifiScanFailures;}
  else ++wifiScanSuccesses;
  out+=",\"networks\":[";nearbyCount=0;
  for(uint8_t i=0;i<wifiNetworkCount;++i){
    if(i)out+=",";
    out+="{\"ssid\":"+quoteJson(wifiNetworks[i].ssid)+",\"rssi\":"+String(wifiNetworks[i].rssi)+
      ",\"channel\":"+String((unsigned)wifiNetworks[i].channel)+"}";
    if(nearbyCount<5)nearbyNames[nearbyCount++]=wifiNetworks[i].ssid;
  }
  out+="]}";wifiStopScan();wifiScanCache=out;wifiCloudPaused.store(false);
  // Resume queued STA work through the normal bounded retry policy.
}
void wifiRequestScan(uint32_t now){
  if(wifiScanBusy()||otaActive||otaRestart)return;
  if(wifiHadScan && !elapsed(now,wifiLastScanRequest,15000U)){
    wifiScanCache="{\"scanning\":false,\"error\":\"scan_cooldown\",\"networks\":[]}";return;
  }
  wifiHadScan=true;wifiLastScanRequest=now;
  wifiScanCache="{\"scanning\":true,\"networks\":[]}";
  wifiLastScanErr=0;wifiLastApCount=wifiLastVisibleCount=0;wifiNetworkCount=0;
  wifiScanTries=0;wifiScanAt=wifiPhaseAt=now;wifiScanPhase=BusaScanPhase::Requested;
  // No radio API here: this function also runs inside HTTP handlers.
}
String wifiScanJSON(){
  return wifiScanBusy()?String("{\"scanning\":true,\"networks\":[]}"):wifiScanCache;
}
String wifiRecoveryJSON(){
  return String("{\"local_priority\":true,\"ap_sta_simultaneous\":true,\"scan_active\":")+(wifiScanBusy()?"true":"false")+
    ",\"connect_pending\":"+(wifiConnectPending?"true":"false")+
    ",\"connecting\":"+(wifiConnecting?"true":"false")+
    ",\"sta_deferred_for_local\":"+(wifiDeferredForLocal?"true":"false")+
    ",\"attempts\":"+String(wifiConnectAttempts)+
    ",\"connect_failures\":"+String(wifiConnectFailures)+
    ",\"scan_failures\":"+String(wifiScanFailures)+
    ",\"scan_successes\":"+String(wifiScanSuccesses)+
    ",\"scan_start_error\":"+String(wifiLastScanErr)+
    ",\"scan_attempt\":"+String((unsigned)wifiScanTries)+
    ",\"scan_channel\":"+String((unsigned)wifiScanChannel)+
    ",\"ap_count\":"+String((unsigned)wifiLastApCount)+
    ",\"visible_ssid_count\":"+String((unsigned)wifiLastVisibleCount)+
    ",\"scan_engine\":\"arduino_async_channel_steps\""+
    ",\"radio_channel\":"+String((unsigned)wifiRadioChannel)+
    ",\"channel_changes\":"+String(wifiChannelChanges)+
    ",\"retry_delay_ms\":"+String(wifiRetryDelay)+
    ",\"disconnects\":"+String(wifiDisconnectCount.load())+
    ",\"last_disconnect_reason\":"+String(wifiDisconnectReason.load())+
    ",\"ap_starts\":"+String(wifiApStarts.load())+
    ",\"ap_stops\":"+String(wifiApStops.load())+
    ",\"ap_client_joins\":"+String(wifiApJoins.load())+
    ",\"ap_client_leaves\":"+String(wifiApLeaves.load())+"}";
}
void wifiQueueRetry(uint32_t now,bool failure){
  wifiConnecting=wifiJoinSettling=false;wifiConnectPending=wifiHasCredentials();
  if(failure){++wifiConnectFailures;if(wifiFailureStreak<4)++wifiFailureStreak;}
  wifiRetryAt=now;
  wifiRetryDelay=failure?((wifiFailureStreak<4)?(30000U<<wifiFailureStreak):300000U):0;
  wifiCloudPaused.store(false);
}
void wifiRecoveryTick(uint32_t now){
  wifiDeferredForLocal=false; // Compatibility field: local clients no longer defer STA.
  // No radio mutation while receiving OTA. prepare rejects an active scan.
  if(otaActive||otaRestart){wifiCloudPaused.store(true);return;}
  if(wifiCancelRequested){wifiCancelRequested=false;wifiCancelScan();}
  if(elapsed(now,wifiLastChannelCheck,1000U) && !wifiScanBusy()){
    wifiLastChannelCheck=now;uint8_t primary=0;wifi_second_chan_t secondary;
    if(esp_wifi_get_channel(&primary,&secondary)==ESP_OK){
      if(wifiRadioChannel && primary!=wifiRadioChannel)++wifiChannelChanges;
      wifiRadioChannel=primary;
    }
  }
  if(wifiScanBusy()){
    wifiCloudPaused.store(true);
    if(elapsed(now,wifiScanAt,BUSA_SCAN_TIMEOUT_MS)){
      wifiFinishScan("scan_timeout");return;
    }
    if(wifiScanPhase==BusaScanPhase::Requested){
      WiFi.setAutoReconnect(false);
      if(wifiConnecting || wifiJoinSettling || WiFi.status()!=WL_CONNECTED){
        WiFi.disconnect(false,false);wifiConnecting=wifiJoinSettling=false;
        wifiConnectPending=wifiHasCredentials();
      }
      wifi_country_t country={};
      if(esp_wifi_get_country(&country)==ESP_OK && country.schan>=1 && country.nchan &&
         country.schan+country.nchan-1<=14){
        wifiScanChannel=country.schan;wifiScanLastChannel=country.schan+country.nchan-1;
      }else{wifiScanChannel=1;wifiScanLastChannel=11;}
      WiFi.scanDelete();wifiPhaseAt=now;wifiScanPhase=BusaScanPhase::Settling;return;
    }
    if(wifiScanPhase==BusaScanPhase::HomePause){
      if(!elapsed(now,wifiPhaseAt,BUSA_SCAN_HOME_MS))return;
      wifiScanPhase=BusaScanPhase::Settling;
      wifiPhaseAt=now-BUSA_SCAN_SETTLE_MS;
    }
    if(wifiScanPhase==BusaScanPhase::Settling){
      if(!elapsed(now,wifiPhaseAt,BUSA_SCAN_SETTLE_MS))return;
      ++wifiScanTries;
      const int rc=WiFi.scanNetworks(true,true,false,120,wifiScanChannel);
      if(rc==WIFI_SCAN_FAILED){
        wifiLastScanErr=rc;
        if(wifiScanTries>=BUSA_SCAN_MAX_TRIES){wifiFinishScan("scan_start_failed");return;}
        // Stop any leftover radio scan; do not start another scan engine.
        esp_wifi_scan_stop();BusaWiFiScanControl::clearScanBusy();
        if(WiFi.status()!=WL_CONNECTED)WiFi.disconnect(false,false);
        WiFi.scanDelete();wifiPhaseAt=now;return;
      }
      wifiLastScanErr=0;wifiScanPhase=BusaScanPhase::Running;return;
    }
    if(wifiScanPhase==BusaScanPhase::Running){
      const int n=WiFi.scanComplete();
      if(n==WIFI_SCAN_RUNNING)return;
      if(n<0){wifiLastScanErr=n;wifiFinishScan("scan_result_failed");return;}
      wifiLastApCount+=(uint16_t)n;
      for(int i=0;i<n;++i){
        const String ssid=WiFi.SSID(i);if(ssid.isEmpty())continue;
        uint8_t k=0;while(k<wifiNetworkCount && wifiNetworks[k].ssid!=ssid)++k;
        if(k>=28)continue;
        if(k==wifiNetworkCount || WiFi.RSSI(i)>wifiNetworks[k].rssi){
          wifiNetworks[k].ssid=ssid;wifiNetworks[k].rssi=WiFi.RSSI(i);
          wifiNetworks[k].channel=(uint8_t)WiFi.channel(i);
        }
        if(k==wifiNetworkCount)++wifiNetworkCount;
      }
      wifiLastVisibleCount=wifiNetworkCount;WiFi.scanDelete();
      // Empty channel is valid, not an error and not a reason to reconnect.
      if(wifiScanChannel>=wifiScanLastChannel){wifiFinishScan();return;}
      ++wifiScanChannel;wifiScanTries=0;wifiPhaseAt=now;
      wifiScanPhase=BusaScanPhase::HomePause;return;
    }
    return;
  }
  const bool connected=WiFi.status()==WL_CONNECTED;
  if(wifiConnecting && connected){
    wifiConnecting=false;wifiFailureStreak=0;wifiCloudPaused.store(false);
  }
  if(wifiConnecting){
    if(elapsed(now,wifiConnectAt,BUSA_CONNECT_TIMEOUT_MS) ||
       wifiDisconnectCount.load()!=wifiAttemptDisconnects){
      WiFi.disconnect(false,false);wifiQueueRetry(now,true);
    }
    return;
  }
  if(connected && !wifiConnectPending){wifiDeferredForLocal=false;wifiCloudPaused.store(false);return;}
  if(!wifiHasCredentials()){wifiConnectPending=false;wifiCloudPaused.store(false);return;}
  if(!connected && !wifiConnectPending)wifiQueueRetry(now,true);
  if(!elapsed(now,wifiPolicyAt,BUSA_BOOT_GRACE_MS) || !elapsed(now,wifiRetryAt,wifiRetryDelay))return;
  if(!wifiJoinSettling){
    WiFi.setAutoReconnect(false);WiFi.disconnect(false,false);
    wifiJoinSettling=true;wifiPhaseAt=now;wifiCloudPaused.store(true);return;
  }
  if(!elapsed(now,wifiPhaseAt,BUSA_SCAN_SETTLE_MS))return;
  wifiJoinSettling=false;wifiConnectPending=false;wifiConnecting=true;
  wifiConnectAt=now;wifiAttemptDisconnects=wifiDisconnectCount.load();++wifiConnectAttempts;
  WiFi.begin(stationSsid.c_str(),stationPass.c_str());
}
