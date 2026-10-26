"""Execute the shipping radio state machine with deterministic Wi-Fi mocks."""
from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
stub=r'''
#include <string>
#include <cstdint>
#include <cassert>
#include <iostream>
class String:public std::string{
public:using std::string::string;String(const std::string&s):std::string(s){}
String(int v):std::string(std::to_string(v)){}
String(unsigned v):std::string(std::to_string(v)){}
bool isEmpty()const{return empty();}
};
const int WL_CONNECTED=3,WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2,WIFI_SCANNING_BIT=1;
const int ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5,ARDUINO_EVENT_WIFI_AP_START=6,
 ARDUINO_EVENT_WIFI_AP_STOP=7,ARDUINO_EVENT_WIFI_AP_STACONNECTED=8,ARDUINO_EVENT_WIFI_AP_STADISCONNECTED=9;
using WiFiEvent_t=int;struct WiFiEventInfo_t{struct{uint8_t reason;}wifi_sta_disconnected;};
bool otaActive=false,otaRestart=false;uint32_t nowMs=0;
uint32_t millis(){return nowMs;}
bool elapsed(uint32_t a,uint32_t b,uint32_t d){return uint32_t(a-b)>=d;}
String quoteJson(const String&s){return "\""+s+"\"";}
String stationSsid="iPhone 16",stationPass="testpass",nearbyNames[5];uint8_t nearbyCount=0;
struct MockWifi{
int statusCode=0,startRc=WIFI_SCAN_RUNNING,complete=WIFI_SCAN_RUNNING;
int scans=0,deletes=0,disconnects=0,begins=0,clients=0,channelNow=1;
String lastSsid,lastPass;
bool autoreconnect=true;
int status(){return statusCode;}
int softAPgetStationNum(){return clients;}
void setAutoReconnect(bool v){assert(!v);autoreconnect=v;}
void disconnect(bool off,bool erase){assert(!off&&!erase);++disconnects;statusCode=0;}
void scanDelete(){++deletes;}
int scanNetworks(bool async,bool hidden,bool passive,int time,uint8_t channel){
 assert(async&&hidden&&!passive&&time==120);++scans;channelNow=channel;return startRc;
}
int scanComplete(){return complete;}
String SSID(int i){return i==0?"":(i==1?"iPhone 16":"iPhone 12");}
int RSSI(int i){return -50-i;}
int channel(int){return channelNow;}
void begin(const char*s,const char*p){assert(!autoreconnect);lastSsid=s;lastPass=p;++begins;}
}WiFi;
using esp_err_t=int;const esp_err_t ESP_OK=0;
using wifi_second_chan_t=int;
struct wifi_country_t{uint8_t schan=1,nchan=3;};
int stopScans=0,clearBits=0,radioChannel=6;
int esp_wifi_scan_stop(){++stopScans;return ESP_OK;}
int esp_wifi_get_channel(uint8_t*p,wifi_second_chan_t*s){*p=radioChannel;*s=0;return ESP_OK;}
int esp_wifi_get_country(wifi_country_t*c){c->schan=1;c->nchan=3;return ESP_OK;}
struct WiFiGenericClass{protected:static void clearStatusBits(int bits){assert(bits==WIFI_SCANNING_BIT);++clearBits;}};
'''
tests=r'''
void tick(uint32_t t){nowMs=t;wifiRecoveryTick(t);}
void completeScan(uint32_t t,int n=3){
 WiFi.complete=n;
 for(int ch=1;ch<=3;ch++){
  tick(t);assert(wifiScanPhase==BusaScanPhase::Running);
  tick(t+1);
  if(ch<3){assert(wifiScanPhase==BusaScanPhase::HomePause);t+=201;}
 }
 assert(!wifiScanBusy());
}
int main(){
 wifiStartPolicy(0);assert(!WiFi.autoreconnect);
 // Saved credentials connect even with an AP client from boot.
 WiFi.clients=1;tick(2999);assert(WiFi.begins==0);
 tick(3000);tick(4199);assert(WiFi.begins==0);
 tick(4200);assert(WiFi.begins==1&&wifiConnecting&&!wifiDeferredForLocal);
 // Additional local clients must never cancel association.
 int d=WiFi.disconnects;WiFi.clients=2;tick(4300);
 assert(wifiConnecting&&WiFi.disconnects==d);
 WiFi.statusCode=WL_CONNECTED;tick(4400);
 assert(!wifiConnecting&&!wifiConnectPending&&!wifiCloudPaused.load());
 // Queue-only discovery with local clients AND a live STA preserves that link.
 nowMs=61000;d=WiFi.disconnects;wifiRequestScan(nowMs);
 assert(WiFi.disconnects==d&&WiFi.scans==0);
 wifiRequestScan(nowMs);assert(WiFi.scans==0);
 tick(61000);tick(61499);assert(WiFi.scans==0);
 completeScan(61500);
 assert(wifiScanSuccesses==1&&wifiLastVisibleCount==2);
 assert(wifiScanJSON().find("iPhone 16")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos);
 assert(WiFi.disconnects==d&&WiFi.statusCode==WL_CONNECTED);
 // A failed start is bounded and not reported as an empty successful scan.
 WiFi.startRc=-2;wifiRequestScan(80000);tick(80000);tick(81200);tick(82400);
 assert(!wifiScanBusy()&&wifiLastScanErr==-2&&wifiScanFailures==1);
 assert(wifiScanJSON().find("scan_start_failed")!=std::string::npos);
 assert(stopScans&&clearBits&&WiFi.statusCode==WL_CONNECTED);
 // Loss of internet retries with local clients present; no retry storm.
 WiFi.statusCode=0;tick(90000);
 assert(wifiRetryDelay==60000&&wifiConnectFailures==1);
 tick(149999);assert(WiFi.begins==1);
 tick(150000);tick(151200);assert(WiFi.begins==2&&WiFi.clients==2);
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});tick(151201);
 assert(wifiRetryDelay==120000&&!wifiConnecting);
 tick(271201);tick(272401);assert(WiFi.begins==3);
 tick(284401);assert(!wifiConnecting&&wifiConnectFailures==3&&wifiRetryDelay==240000);
 // Saving credentials bypasses previous backoff but lets HTTP response finish.
 d=WiFi.disconnects;wifiScheduleConnect(300000);
 assert(WiFi.disconnects==d&&WiFi.begins==3);
 tick(301499);assert(WiFi.disconnects==d);
 tick(301500);tick(302700);assert(WiFi.begins==4&&wifiConnecting);
 // A second save during association retargets exactly once through the loop.
 stationSsid="iPhone 12";stationPass="newtestpass";d=WiFi.disconnects;
 wifiScheduleConnect(302750);assert(WiFi.disconnects==d&&WiFi.begins==4);
 tick(304250);tick(305450);
 assert(WiFi.begins==5&&WiFi.lastSsid==stationSsid&&WiFi.lastPass==stationPass);
 WiFi.statusCode=WL_CONNECTED;tick(305451);
 assert(!wifiConnecting&&wifiFailureStreak==0&&!wifiDeferredForLocal);
 // IDF reports migrated home channel; controller observes it without a reset.
 radioChannel=11;tick(306800);assert(wifiRadioChannel==11&&wifiChannelChanges==1);
 // Working AP+STA remains untouched over many polls; no forced channel reset.
 d=WiFi.disconnects;
 for(uint32_t t=307000;t<310000;t+=200)tick(t);
 assert(WiFi.begins==5&&WiFi.disconnects==d);
 // Scan timeout stops radio scan and releases Arduino busy bits.
 WiFi.startRc=-1;WiFi.complete=-1;wifiRequestScan(400000);tick(400000);tick(401200);
 int stops=stopScans;tick(412000);assert(!wifiScanBusy()&&stopScans==stops+1);
 assert(wifiScanJSON().find("scan_timeout")!=std::string::npos);
 wifiRequestScan(430000);tick(430000);tick(431200);
 wifiCancelRequested=true;tick(431201);assert(!wifiScanBusy());
 assert(wifiScanJSON().find("scan_cancelled")!=std::string::npos);
 // OTA locks all new radio mutations.
 otaActive=true;int scans=WiFi.scans,joins=WiFi.begins;d=WiFi.disconnects;
 wifiRequestScan(450000);wifiScheduleConnect(450000);tick(455000);
 assert(!wifiScanBusy()&&WiFi.scans==scans&&WiFi.begins==joins&&WiFi.disconnects==d);
 otaActive=false;
 // Millis rollover during response grace with a local AP client attached.
 WiFi.statusCode=0;wifiStartPolicy(UINT32_MAX-20000);wifiScheduleConnect(UINT32_MAX-1000);
 tick(498);assert(WiFi.begins==joins);tick(499);tick(1699);
 assert(WiFi.begins==joins+1&&wifiConnecting);
 // Empty scans are valid; cooldown does no radio work.
 WiFi.statusCode=WL_CONNECTED;wifiConnectPending=false;wifiConnecting=false;
 wifiHadScan=false;wifiRequestScan(20000);tick(20000);completeScan(21200,0);
 assert(wifiLastVisibleCount==0&&wifiScanJSON().find("error")==std::string::npos);
 scans=WiFi.scans;wifiRequestScan(22000);assert(!wifiScanBusy()&&WiFi.scans==scans);
 assert(wifiScanJSON().find("scan_cooldown")!=std::string::npos);
 // Repeated failures cap at five minutes even while local clients stay attached.
 WiFi.statusCode=0;wifiScheduleConnect(600000);uint32_t retry=601500;
 for(int i=0;i<5;++i){
  tick(retry);tick(retry+1200);assert(wifiConnecting);
  wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});tick(retry+1201);
  assert(!wifiConnecting&&wifiRetryDelay<=300000);
  retry=wifiRetryAt+wifiRetryDelay;
 }
 assert(wifiRetryDelay==300000);
 stationPass="";joins=WiFi.begins;wifiScheduleConnect(retry);tick(retry+5000);
 assert(!wifiConnectPending&&WiFi.begins==joins&&!wifiCloudPaused.load());
 assert(wifiRecoveryJSON().find("ap_sta_simultaneous")!=std::string::npos);
 std::cout<<"PASS: shipped AP+STA: local clients during boot/join/retry, HTTP grace, -2, 201, backoff, OTA, scans, rollover"<<std::endl;
}

'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'test.cpp';p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n'+tests)
    exe=Path(d)/'test'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
