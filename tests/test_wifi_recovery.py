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
void begin(const char*,const char*){assert(!autoreconnect);++begins;}
}WiFi;
using esp_err_t=int;const esp_err_t ESP_OK=0;
using wifi_second_chan_t=int;
struct wifi_country_t{uint8_t schan=1,nchan=3;};
int stopScans=0,clearBits=0;
int esp_wifi_scan_stop(){++stopScans;return ESP_OK;}
int esp_wifi_get_channel(uint8_t*p,wifi_second_chan_t*s){*p=6;*s=0;return ESP_OK;}
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
 // AP client wins over boot saved credentials indefinitely.
 WiFi.clients=1;tick(16000);tick(60000);assert(WiFi.begins==0&&wifiDeferredForLocal);
 // Queue-only HTTP request; discovery still works with a local client attached.
 nowMs=61000;int d=WiFi.disconnects;wifiRequestScan(nowMs);
 assert(WiFi.disconnects==d&&WiFi.scans==0);
 wifiRequestScan(nowMs);assert(WiFi.scans==0);
 tick(61000);tick(62199);assert(WiFi.scans==0);
 completeScan(62200);
 assert(wifiScanSuccesses==1&&wifiLastVisibleCount==2);
 assert(wifiScanJSON().find("iPhone 16")!=std::string::npos);
 assert(wifiScanJSON().find("[,")==std::string::npos);
 tick(70000);assert(WiFi.begins==0&&!WiFi.autoreconnect);
 // A failed start is retried finitely, and never reported as an empty successful scan.
 WiFi.startRc=-2;wifiRequestScan(80000);tick(80000);tick(81200);tick(82400);
 assert(!wifiScanBusy()&&wifiLastScanErr==-2&&wifiScanFailures==1);
 assert(wifiScanJSON().find("scan_start_failed")!=std::string::npos);
 assert(stopScans&&clearBits);
 // Client leaves: 10 s quiet interval + 1.2 s settling before a single association.
 WiFi.clients=0;tick(90000);assert(WiFi.begins==0);
 tick(92400);assert(WiFi.begins==0);tick(93600);assert(WiFi.begins==1&&wifiConnecting);
 // Local client arriving during association cancels it without resetting the AP.
 WiFi.clients=1;tick(93700);assert(!wifiConnecting&&wifiConnectPending);
 tick(190000);assert(WiFi.begins==1);
 // Existing good STA connection is preserved with a local client, including during scans.
 WiFi.statusCode=WL_CONNECTED;wifiConnectPending=false;tick(191000);
 WiFi.startRc=-1;int before=WiFi.disconnects;wifiRequestScan(192000);tick(192000);
 completeScan(193200);assert(WiFi.disconnects==before&&WiFi.statusCode==WL_CONNECTED);
 tick(194000);assert(!wifiConnectPending&&!wifiConnecting);
 // With no local client, failure 201 imposes a 60 s backoff; timeout then 120 s.
 WiFi.clients=0;WiFi.statusCode=0;tick(200000);
 assert(wifiRetryDelay==60000&&wifiConnectFailures==1);
 tick(259999);assert(WiFi.begins==1);
 tick(260000);tick(261200);assert(WiFi.begins==2);
 wifiRecoveryEvent(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{{201}});tick(261201);
 assert(wifiRetryDelay==120000&&!wifiConnecting);
 tick(381201);tick(382401);assert(WiFi.begins==3);
 tick(394401);assert(!wifiConnecting&&wifiConnectFailures==3&&wifiRetryDelay==240000);
 // Asynchronous scan timeout stops the radio scan and releases Arduino busy bits.
 WiFi.complete=-1;wifiRequestScan(400000);tick(400000);tick(401200);
 int stops=stopScans;tick(412000);assert(!wifiScanBusy()&&stopScans==stops+1);
 assert(wifiScanJSON().find("scan_timeout")!=std::string::npos);
 // Cancel is handled by the loop, does not resume STA while client is present.
 wifiRequestScan(430000);tick(430000);tick(431200);
 WiFi.clients=1;wifiCancelRequested=true;tick(431201);assert(!wifiScanBusy());
 assert(wifiScanJSON().find("scan_cancelled")!=std::string::npos);
 // OTA rejection/lock: no new scan, association, or disconnect during upload.
 otaActive=true;int scans=WiFi.scans,joins=WiFi.begins;before=WiFi.disconnects;
 wifiRequestScan(450000);tick(450000);
 assert(!wifiScanBusy()&&WiFi.scans==scans&&WiFi.begins==joins&&WiFi.disconnects==before);
 otaActive=false;
 // Unsigned arithmetic handles millis rollover in the quiet/boot/settle checks.
 WiFi.clients=0;wifiStartPolicy(UINT32_MAX-20000);wifiScheduleConnect(UINT32_MAX-2000);
 tick(0);tick(1200);assert(WiFi.begins==joins+1);
 // Empty scans are valid; repeated request during cooldown performs no radio work.
 WiFi.statusCode=WL_CONNECTED;wifiConnectPending=false;wifiConnecting=false;
 wifiHadScan=false;wifiRequestScan(20000);tick(20000);completeScan(21200,0);
 assert(wifiLastVisibleCount==0&&wifiScanJSON().find("error")==std::string::npos);
 scans=WiFi.scans;wifiRequestScan(22000);assert(!wifiScanBusy()&&WiFi.scans==scans);
 assert(wifiScanJSON().find("scan_cooldown")!=std::string::npos);
 std::cout<<"PASS: shipped radio controller: local priority, async scans, -2, 201, retries, OTA, cancellation, rollover"<<std::endl;
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'test.cpp';p.write_text(stub+'\n#include "BUSA_WifiRecovery.h"\n'+tests)
    exe=Path(d)/'test'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
